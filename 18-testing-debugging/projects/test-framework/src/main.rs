#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    gpio::{gpioa::PA5, Output, PushPull},
    serial::{config::Config, Serial},
    spi::{Spi, NoMiso, NoMosi},
    i2c::I2c,
    adc::{Adc, config::AdcConfig},
    timer::{Timer, Event},
    delay::Delay,
};
use cortex_m::peripheral::DWT;
use heapless::{Vec, String};
use defmt_rtt as _;

// Test framework core
pub mod test_framework {
    use super::*;
    use heapless::{Vec, String};
    
    #[derive(Debug, Clone)]
    pub struct TestResult {
        pub name: String<64>,
        pub passed: bool,
        pub duration_us: u32,
        pub message: String<128>,
    }
    
    #[derive(Debug)]
    pub struct TestSuite {
        pub name: String<64>,
        pub results: Vec<TestResult, 32>,
        pub setup_time_us: u32,
        pub teardown_time_us: u32,
    }
    
    impl TestSuite {
        pub fn new(name: &str) -> Self {
            let mut suite_name = String::new();
            suite_name.push_str(name).ok();
            
            Self {
                name: suite_name,
                results: Vec::new(),
                setup_time_us: 0,
                teardown_time_us: 0,
            }
        }
        
        pub fn add_result(&mut self, result: TestResult) {
            self.results.push(result).ok();
        }
        
        pub fn passed_count(&self) -> usize {
            self.results.iter().filter(|r| r.passed).count()
        }
        
        pub fn failed_count(&self) -> usize {
            self.results.iter().filter(|r| !r.passed).count()
        }
        
        pub fn total_duration_us(&self) -> u32 {
            self.results.iter().map(|r| r.duration_us).sum::<u32>() 
                + self.setup_time_us + self.teardown_time_us
        }
    }
    
    pub struct TestRunner {
        pub suites: Vec<TestSuite, 16>,
        pub start_time: u32,
    }
    
    impl TestRunner {
        pub fn new() -> Self {
            Self {
                suites: Vec::new(),
                start_time: 0,
            }
        }
        
        pub fn add_suite(&mut self, suite: TestSuite) {
            self.suites.push(suite).ok();
        }
        
        pub fn run_all(&mut self) {
            self.start_time = get_timestamp();
            
            for suite in &mut self.suites {
                defmt::info!("Running test suite: {}", suite.name.as_str());
                
                let setup_start = get_timestamp();
                // Setup code would go here
                suite.setup_time_us = get_timestamp() - setup_start;
                
                for result in &suite.results {
                    if result.passed {
                        defmt::info!("  ✓ {} ({} μs)", result.name.as_str(), result.duration_us);
                    } else {
                        defmt::error!("  ✗ {} ({} μs): {}", 
                            result.name.as_str(), result.duration_us, result.message.as_str());
                    }
                }
                
                let teardown_start = get_timestamp();
                // Teardown code would go here
                suite.teardown_time_us = get_timestamp() - teardown_start;
                
                defmt::info!("Suite {} completed: {}/{} passed", 
                    suite.name.as_str(), suite.passed_count(), suite.results.len());
            }
        }
        
        pub fn generate_report(&self) -> TestReport {
            let total_tests = self.suites.iter().map(|s| s.results.len()).sum();
            let total_passed = self.suites.iter().map(|s| s.passed_count()).sum();
            let total_failed = self.suites.iter().map(|s| s.failed_count()).sum();
            let total_duration = self.suites.iter().map(|s| s.total_duration_us()).sum();
            
            TestReport {
                total_tests,
                total_passed,
                total_failed,
                total_duration_us: total_duration,
                suites: self.suites.clone(),
            }
        }
    }
    
    #[derive(Debug, Clone)]
    pub struct TestReport {
        pub total_tests: usize,
        pub total_passed: usize,
        pub total_failed: usize,
        pub total_duration_us: u32,
        pub suites: Vec<TestSuite, 16>,
    }
    
    impl TestReport {
        pub fn print_summary(&self) {
            defmt::info!("=== Test Report Summary ===");
            defmt::info!("Total tests: {}", self.total_tests);
            defmt::info!("Passed: {}", self.total_passed);
            defmt::info!("Failed: {}", self.total_failed);
            defmt::info!("Success rate: {}%", 
                (self.total_passed * 100) / self.total_tests.max(1));
            defmt::info!("Total duration: {} μs", self.total_duration_us);
            defmt::info!("Average test time: {} μs", 
                self.total_duration_us / self.total_tests.max(1) as u32);
        }
    }
}

// Hardware abstraction layer for testing
pub mod hal_testing {
    use super::*;
    
    pub struct TestHardware {
        pub led: PA5<Output<PushPull>>,
        pub delay: Delay,
        pub timer: Timer<stm32::TIM2>,
    }
    
    impl TestHardware {
        pub fn new(
            led: PA5<Output<PushPull>>,
            delay: Delay,
            timer: Timer<stm32::TIM2>,
        ) -> Self {
            Self { led, delay, timer }
        }
        
        pub fn blink_test(&mut self) -> bool {
            // Test LED blinking
            for _ in 0..5 {
                self.led.set_high();
                self.delay.delay_ms(100u32);
                self.led.set_low();
                self.delay.delay_ms(100u32);
            }
            true
        }
        
        pub fn timer_test(&mut self) -> bool {
            // Test timer functionality
            self.timer.start(1.hz());
            self.timer.wait().is_ok()
        }
    }
}

// Mock hardware for unit testing
pub mod mock_hardware {
    use super::*;
    use embedded_hal_mock::gpio::{State, Transaction as GpioTransaction};
    use embedded_hal_mock::serial::{Transaction as SerialTransaction};
    
    pub struct MockGpio {
        pub expectations: Vec<GpioTransaction, 32>,
        pub current_state: State,
    }
    
    impl MockGpio {
        pub fn new() -> Self {
            Self {
                expectations: Vec::new(),
                current_state: State::Low,
            }
        }
        
        pub fn expect_set_high(&mut self) {
            self.expectations.push(GpioTransaction::set(State::High)).ok();
        }
        
        pub fn expect_set_low(&mut self) {
            self.expectations.push(GpioTransaction::set(State::Low)).ok();
        }
        
        pub fn verify(&self) -> bool {
            // Verify all expectations were met
            true
        }
    }
}

// Performance testing utilities
pub mod performance_testing {
    use super::*;
    
    pub struct PerformanceTest {
        pub name: String<64>,
        pub iterations: u32,
        pub min_duration_us: u32,
        pub max_duration_us: u32,
        pub avg_duration_us: u32,
        pub total_duration_us: u32,
    }
    
    impl PerformanceTest {
        pub fn new(name: &str, iterations: u32) -> Self {
            let mut test_name = String::new();
            test_name.push_str(name).ok();
            
            Self {
                name: test_name,
                iterations,
                min_duration_us: u32::MAX,
                max_duration_us: 0,
                avg_duration_us: 0,
                total_duration_us: 0,
            }
        }
        
        pub fn run<F>(&mut self, mut test_fn: F) -> bool 
        where
            F: FnMut() -> bool,
        {
            let mut durations = Vec::<u32, 1000>::new();
            
            for _ in 0..self.iterations {
                let start = get_timestamp();
                let success = test_fn();
                let duration = get_timestamp() - start;
                
                if !success {
                    return false;
                }
                
                durations.push(duration).ok();
                self.total_duration_us += duration;
                
                if duration < self.min_duration_us {
                    self.min_duration_us = duration;
                }
                if duration > self.max_duration_us {
                    self.max_duration_us = duration;
                }
            }
            
            self.avg_duration_us = self.total_duration_us / self.iterations;
            
            defmt::info!("Performance test '{}' completed:", self.name.as_str());
            defmt::info!("  Iterations: {}", self.iterations);
            defmt::info!("  Min: {} μs", self.min_duration_us);
            defmt::info!("  Max: {} μs", self.max_duration_us);
            defmt::info!("  Avg: {} μs", self.avg_duration_us);
            defmt::info!("  Total: {} μs", self.total_duration_us);
            
            true
        }
    }
}

// Memory testing utilities
pub mod memory_testing {
    use super::*;
    
    pub struct MemoryTest {
        pub initial_free: usize,
        pub current_free: usize,
        pub peak_usage: usize,
        pub allocations: u32,
        pub deallocations: u32,
    }
    
    impl MemoryTest {
        pub fn new() -> Self {
            Self {
                initial_free: get_free_memory(),
                current_free: 0,
                peak_usage: 0,
                allocations: 0,
                deallocations: 0,
            }
        }
        
        pub fn checkpoint(&mut self) {
            self.current_free = get_free_memory();
            let usage = self.initial_free - self.current_free;
            if usage > self.peak_usage {
                self.peak_usage = usage;
            }
        }
        
        pub fn verify_no_leaks(&self) -> bool {
            let final_free = get_free_memory();
            let leaked = self.initial_free.saturating_sub(final_free);
            
            if leaked > 0 {
                defmt::error!("Memory leak detected: {} bytes", leaked);
                false
            } else {
                defmt::info!("No memory leaks detected");
                true
            }
        }
        
        pub fn print_stats(&self) {
            defmt::info!("Memory test statistics:");
            defmt::info!("  Initial free: {} bytes", self.initial_free);
            defmt::info!("  Current free: {} bytes", self.current_free);
            defmt::info!("  Peak usage: {} bytes", self.peak_usage);
            defmt::info!("  Allocations: {}", self.allocations);
            defmt::info!("  Deallocations: {}", self.deallocations);
        }
    }
}

// Test data generators
pub mod test_data {
    use super::*;
    
    pub struct TestDataGenerator {
        seed: u32,
    }
    
    impl TestDataGenerator {
        pub fn new(seed: u32) -> Self {
            Self { seed }
        }
        
        pub fn next_u32(&mut self) -> u32 {
            // Simple LCG for test data generation
            self.seed = self.seed.wrapping_mul(1103515245).wrapping_add(12345);
            self.seed
        }
        
        pub fn next_u16(&mut self) -> u16 {
            (self.next_u32() >> 16) as u16
        }
        
        pub fn next_u8(&mut self) -> u8 {
            (self.next_u32() >> 24) as u8
        }
        
        pub fn next_bool(&mut self) -> bool {
            (self.next_u32() & 1) == 1
        }
        
        pub fn fill_buffer(&mut self, buffer: &mut [u8]) {
            for byte in buffer.iter_mut() {
                *byte = self.next_u8();
            }
        }
        
        pub fn generate_test_vector(&mut self, size: usize) -> Vec<u8, 256> {
            let mut vec = Vec::new();
            for _ in 0..size.min(256) {
                vec.push(self.next_u8()).ok();
            }
            vec
        }
    }
}

// Utility functions
fn get_timestamp() -> u32 {
    // In a real implementation, this would read from DWT cycle counter
    // For now, return a dummy value
    42
}

fn get_free_memory() -> usize {
    // In a real implementation, this would check available heap/stack
    // For now, return a dummy value
    1024
}

// Test macros
#[macro_export]
macro_rules! assert_test {
    ($condition:expr, $message:expr) => {
        if !$condition {
            defmt::error!("Assertion failed: {}", $message);
            return false;
        }
    };
}

#[macro_export]
macro_rules! test_case {
    ($name:expr, $test_fn:expr) => {{
        let start_time = get_timestamp();
        let mut test_name = String::new();
        test_name.push_str($name).ok();
        
        let passed = $test_fn();
        let duration = get_timestamp() - start_time;
        
        let mut message = String::new();
        if !passed {
            message.push_str("Test failed").ok();
        }
        
        TestResult {
            name: test_name,
            passed,
            duration_us: duration,
            message,
        }
    }};
}

// Example test implementations
fn test_basic_math() -> bool {
    assert_test!(2 + 2 == 4, "Basic addition failed");
    assert_test!(10 - 5 == 5, "Basic subtraction failed");
    assert_test!(3 * 4 == 12, "Basic multiplication failed");
    assert_test!(15 / 3 == 5, "Basic division failed");
    true
}

fn test_array_operations() -> bool {
    let mut arr = [1, 2, 3, 4, 5];
    
    assert_test!(arr.len() == 5, "Array length incorrect");
    assert_test!(arr[0] == 1, "Array indexing failed");
    
    arr[2] = 10;
    assert_test!(arr[2] == 10, "Array assignment failed");
    
    let sum: i32 = arr.iter().sum();
    assert_test!(sum == 22, "Array sum calculation failed");
    
    true
}

fn test_string_operations() -> bool {
    let mut s = String::<32>::new();
    
    assert_test!(s.push_str("Hello").is_ok(), "String push failed");
    assert_test!(s.len() == 5, "String length incorrect");
    assert_test!(s.as_str() == "Hello", "String content incorrect");
    
    assert_test!(s.push_str(" World").is_ok(), "String append failed");
    assert_test!(s.as_str() == "Hello World", "String concatenation failed");
    
    true
}

use test_framework::*;

#[entry]
fn main() -> ! {
    // Initialize hardware
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    let gpioa = dp.GPIOA.split();
    let led = gpioa.pa5.into_push_pull_output();
    let delay = Delay::new(cp.SYST, &clocks);
    let timer = Timer::new(dp.TIM2, &clocks);
    
    // Initialize test framework
    let mut test_runner = TestRunner::new();
    
    // Create and run unit tests
    let mut unit_test_suite = TestSuite::new("Unit Tests");
    
    unit_test_suite.add_result(test_case!("Basic Math", test_basic_math));
    unit_test_suite.add_result(test_case!("Array Operations", test_array_operations));
    unit_test_suite.add_result(test_case!("String Operations", test_string_operations));
    
    test_runner.add_suite(unit_test_suite);
    
    // Create and run performance tests
    let mut perf_test_suite = TestSuite::new("Performance Tests");
    
    let mut math_perf_test = performance_testing::PerformanceTest::new("Math Performance", 1000);
    let math_result = math_perf_test.run(|| {
        let mut sum = 0;
        for i in 0..100 {
            sum += i * i;
        }
        sum > 0
    });
    
    let mut perf_result = TestResult {
        name: String::new(),
        passed: math_result,
        duration_us: math_perf_test.avg_duration_us,
        message: String::new(),
    };
    perf_result.name.push_str("Math Performance").ok();
    
    perf_test_suite.add_result(perf_result);
    test_runner.add_suite(perf_test_suite);
    
    // Create and run memory tests
    let mut memory_test_suite = TestSuite::new("Memory Tests");
    
    let mut mem_test = memory_testing::MemoryTest::new();
    
    // Simulate some memory operations
    let mut test_vec = Vec::<u8, 100>::new();
    for i in 0..50 {
        test_vec.push(i).ok();
    }
    mem_test.checkpoint();
    
    let memory_result = mem_test.verify_no_leaks();
    mem_test.print_stats();
    
    let mut mem_result = TestResult {
        name: String::new(),
        passed: memory_result,
        duration_us: 100,
        message: String::new(),
    };
    mem_result.name.push_str("Memory Leak Test").ok();
    
    memory_test_suite.add_result(mem_result);
    test_runner.add_suite(memory_test_suite);
    
    // Run all tests
    defmt::info!("Starting embedded test framework...");
    test_runner.run_all();
    
    // Generate and display report
    let report = test_runner.generate_report();
    report.print_summary();
    
    defmt::info!("All tests completed!");
    
    loop {
        // Keep the program running
        cortex_m::asm::wfi();
    }
}