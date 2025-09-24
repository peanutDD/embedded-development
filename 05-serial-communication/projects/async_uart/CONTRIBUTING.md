# Contributing to Async UART

We welcome contributions to the Async UART project! This document provides guidelines for contributing to the project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Contributing Process](#contributing-process)
- [Coding Standards](#coding-standards)
- [Testing Guidelines](#testing-guidelines)
- [Documentation](#documentation)
- [Submitting Changes](#submitting-changes)

## Code of Conduct

This project adheres to a code of conduct that we expect all contributors to follow. Please be respectful and constructive in all interactions.

## Getting Started

### Prerequisites

- Rust 1.70.0 or later
- Cargo
- Git
- Basic knowledge of embedded Rust development
- Familiarity with async/await programming

### Development Setup

1. **Fork and Clone**
   ```bash
   git clone https://github.com/your-username/async-uart.git
   cd async-uart
   ```

2. **Install Dependencies**
   ```bash
   # Install Rust if not already installed
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   
   # Install embedded targets
   rustup target add thumbv7em-none-eabihf
   rustup target add thumbv6m-none-eabi
   ```

3. **Verify Setup**
   ```bash
   cargo check --features std-support
   cargo test --features std-support
   ```

## Contributing Process

### 1. Issue Discussion

- Check existing issues before creating new ones
- For new features, create an issue to discuss the proposal
- For bugs, provide detailed reproduction steps
- Use appropriate issue labels

### 2. Branch Strategy

- Create feature branches from `main`
- Use descriptive branch names: `feature/add-spi-protocol`, `fix/buffer-overflow`
- Keep branches focused on single features or fixes

### 3. Development Workflow

```bash
# Create and switch to feature branch
git checkout -b feature/your-feature-name

# Make changes and commit
git add .
git commit -m "Add: descriptive commit message"

# Push to your fork
git push origin feature/your-feature-name

# Create pull request
```

## Coding Standards

### Rust Style Guidelines

- Follow the official [Rust Style Guide](https://doc.rust-lang.org/nightly/style-guide/)
- Use `rustfmt` for code formatting
- Use `clippy` for linting
- Maintain consistent naming conventions

### Code Organization

- **Modules**: Keep modules focused and well-organized
- **Functions**: Write small, focused functions with clear purposes
- **Documentation**: Document all public APIs with examples
- **Error Handling**: Use appropriate error types and handling strategies

### Example Code Style

```rust
/// Processes incoming UART data asynchronously.
/// 
/// # Arguments
/// 
/// * `buffer` - The buffer containing incoming data
/// * `config` - UART configuration parameters
/// 
/// # Returns
/// 
/// Returns `Ok(bytes_processed)` on success, or an error if processing fails.
/// 
/// # Examples
/// 
/// ```rust
/// let result = process_uart_data(&buffer, &config).await?;
/// println!("Processed {} bytes", result);
/// ```
pub async fn process_uart_data(
    buffer: &[u8], 
    config: &UartConfig
) -> Result<usize, AsyncUartError> {
    // Implementation here
}
```

## Testing Guidelines

### Test Categories

1. **Unit Tests**: Test individual functions and modules
2. **Integration Tests**: Test component interactions
3. **Stress Tests**: Test under high load conditions
4. **Platform Tests**: Test platform-specific functionality

### Writing Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use tokio_test;

    #[tokio::test]
    async fn test_uart_basic_operation() {
        let config = UartConfig::default();
        let mut uart = AsyncUart::new(config).unwrap();
        
        // Test implementation
        let result = uart.write(b"test").await;
        assert!(result.is_ok());
    }

    #[test]
    fn test_config_validation() {
        let config = UartConfigBuilder::new()
            .baud_rate(9600)
            .build();
        
        assert!(config.is_ok());
    }
}
```

### Running Tests

```bash
# Run all tests
cargo test --features std-support

# Run specific test category
cargo test integration_tests --features std-support
cargo test stress_tests --features std-support

# Run benchmarks
cargo bench

# Run with coverage (requires cargo-tarpaulin)
cargo tarpaulin --features std-support
```

## Documentation

### Documentation Requirements

- **Public APIs**: All public functions, structs, and modules must be documented
- **Examples**: Include usage examples in documentation
- **Error Cases**: Document possible error conditions
- **Safety**: Document any unsafe code usage

### Documentation Style

```rust
/// Brief description of the function.
/// 
/// Longer description providing more context about the function's
/// purpose, behavior, and usage patterns.
/// 
/// # Arguments
/// 
/// * `param1` - Description of the first parameter
/// * `param2` - Description of the second parameter
/// 
/// # Returns
/// 
/// Description of the return value and its meaning.
/// 
/// # Errors
/// 
/// This function will return an error if:
/// - Condition 1 occurs
/// - Condition 2 happens
/// 
/// # Examples
/// 
/// ```rust
/// use async_uart::*;
/// 
/// let result = function_name(param1, param2)?;
/// assert_eq!(result, expected_value);
/// ```
/// 
/// # Safety
/// 
/// (Only for unsafe functions)
/// Describe safety requirements and invariants.
```

### Building Documentation

```bash
# Generate documentation
cargo doc --features std-support --no-deps --open

# Check documentation links
cargo doc --features std-support --no-deps
```

## Submitting Changes

### Pull Request Guidelines

1. **Title**: Use clear, descriptive titles
2. **Description**: Explain what changes were made and why
3. **Testing**: Describe how the changes were tested
4. **Breaking Changes**: Clearly mark any breaking changes
5. **Documentation**: Update documentation as needed

### Pull Request Template

```markdown
## Description
Brief description of the changes made.

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update

## Testing
- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Stress tests pass (if applicable)
- [ ] Manual testing performed

## Checklist
- [ ] Code follows the project's style guidelines
- [ ] Self-review of code completed
- [ ] Code is commented, particularly in hard-to-understand areas
- [ ] Documentation updated
- [ ] No new warnings introduced
```

### Review Process

1. **Automated Checks**: CI/CD pipeline runs automatically
2. **Code Review**: Maintainers review code for quality and correctness
3. **Testing**: Verify that all tests pass
4. **Documentation**: Ensure documentation is complete and accurate
5. **Approval**: At least one maintainer approval required

## Platform-Specific Contributions

### Adding New Platform Support

1. **HAL Adapter**: Implement the `HalAdapter` trait for the new platform
2. **Configuration**: Add platform-specific configuration options
3. **Testing**: Create platform-specific tests
4. **Documentation**: Update platform support documentation
5. **Examples**: Provide platform-specific examples

### Platform Testing

```bash
# Test specific platform features
cargo test --features stm32
cargo test --features esp32
cargo test --features rp2040
cargo test --features nrf52
```

## Performance Contributions

### Benchmarking

- Use `criterion` for performance benchmarks
- Include before/after performance comparisons
- Test on relevant hardware when possible
- Document performance characteristics

### Optimization Guidelines

- Profile before optimizing
- Measure the impact of changes
- Consider memory usage in embedded contexts
- Maintain code readability

## Security Considerations

### Security Review

- Review code for potential security vulnerabilities
- Use secure coding practices
- Validate all inputs
- Handle sensitive data appropriately

### Reporting Security Issues

For security-related issues, please email security@embedded-rust.com instead of creating public issues.

## Getting Help

### Communication Channels

- **GitHub Issues**: For bug reports and feature requests
- **GitHub Discussions**: For questions and general discussion
- **Documentation**: Check the user guide and API reference

### Maintainer Contact

- **Primary Maintainer**: dev@embedded-rust.com
- **Response Time**: We aim to respond within 48 hours

## Recognition

Contributors will be recognized in:
- CHANGELOG.md for significant contributions
- README.md contributors section
- Release notes for major features

Thank you for contributing to Async UART! Your contributions help make embedded Rust development better for everyone.