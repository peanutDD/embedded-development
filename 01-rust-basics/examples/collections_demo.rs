// 集合类型演示程序
// 展示Rust的Vector、HashMap、HashSet等集合类型的使用

use std::collections::{HashMap, HashSet, BTreeMap, BTreeSet, VecDeque, LinkedList};

fn main() {
    println!("=== Rust 集合类型演示 ===\n");
    
    // 1. Vector动态数组
    vector_demo();
    
    // 2. HashMap哈希映射
    hashmap_demo();
    
    // 3. HashSet哈希集合
    hashset_demo();
    
    // 4. 其他集合类型
    other_collections_demo();
    
    // 5. 集合操作技巧
    collection_techniques_demo();
    
    // 6. 性能比较
    performance_demo();
}

fn vector_demo() {
    println!("1. Vector动态数组演示:");
    
    // 创建Vector
    let mut numbers = Vec::new();
    numbers.push(1);
    numbers.push(2);
    numbers.push(3);
    println!("  初始vector: {:?}", numbers);
    
    // 使用宏创建
    let mut fruits = vec!["苹果", "香蕉", "橙子"];
    println!("  水果vector: {:?}", fruits);
    
    // 访问元素
    println!("  第一个水果: {}", fruits[0]);
    match fruits.get(1) {
        Some(fruit) => println!("  第二个水果: {}", fruit),
        None => println!("  没有第二个水果"),
    }
    
    // 修改元素
    fruits[2] = "葡萄";
    fruits.push("草莓");
    println!("  修改后: {:?}", fruits);
    
    // 删除元素
    let last = fruits.pop();
    println!("  弹出的元素: {:?}", last);
    println!("  剩余水果: {:?}", fruits);
    
    // 插入和删除
    fruits.insert(1, "芒果");
    println!("  插入芒果后: {:?}", fruits);
    
    let removed = fruits.remove(2);
    println!("  移除的元素: {}", removed);
    println!("  最终水果: {:?}", fruits);
    
    // 迭代
    println!("  遍历水果:");
    for (index, fruit) in fruits.iter().enumerate() {
        println!("    {}: {}", index, fruit);
    }
    
    // Vector容量管理
    let mut capacity_demo = Vec::with_capacity(10);
    println!("  初始容量: {}, 长度: {}", capacity_demo.capacity(), capacity_demo.len());
    
    for i in 0..5 {
        capacity_demo.push(i);
    }
    println!("  添加5个元素后 - 容量: {}, 长度: {}", 
             capacity_demo.capacity(), capacity_demo.len());
    
    capacity_demo.shrink_to_fit();
    println!("  收缩后 - 容量: {}, 长度: {}", 
             capacity_demo.capacity(), capacity_demo.len());
    
    // 切片操作
    let slice = &capacity_demo[1..4];
    println!("  切片[1..4]: {:?}", slice);
    
    println!();
}

fn hashmap_demo() {
    println!("2. HashMap哈希映射演示:");
    
    // 创建HashMap
    let mut scores = HashMap::new();
    scores.insert("Alice", 95);
    scores.insert("Bob", 87);
    scores.insert("Charlie", 92);
    
    println!("  学生成绩: {:?}", scores);
    
    // 访问值
    match scores.get("Alice") {
        Some(score) => println!("  Alice的成绩: {}", score),
        None => println!("  找不到Alice的成绩"),
    }
    
    // 更新值
    scores.insert("Alice", 98); // 覆盖
    println!("  更新Alice成绩后: {:?}", scores);
    
    // 只在键不存在时插入
    scores.entry("David").or_insert(85);
    scores.entry("Alice").or_insert(80); // 不会覆盖
    println!("  使用entry后: {:?}", scores);
    
    // 基于旧值更新
    let alice_score = scores.entry("Alice").or_insert(0);
    *alice_score += 2;
    println!("  Alice加分后: {:?}", scores);
    
    // 遍历
    println!("  所有成绩:");
    for (name, score) in &scores {
        println!("    {}: {}", name, score);
    }
    
    // 统计单词频率示例
    let text = "hello world hello rust world rust programming";
    let mut word_count = HashMap::new();
    
    for word in text.split_whitespace() {
        let count = word_count.entry(word).or_insert(0);
        *count += 1;
    }
    
    println!("  单词频率统计:");
    for (word, count) in &word_count {
        println!("    {}: {}", word, count);
    }
    
    // 从Vector创建HashMap
    let teams = vec!["Blue", "Yellow"];
    let initial_scores = vec![10, 50];
    let team_scores: HashMap<_, _> = teams.iter().zip(initial_scores.iter()).collect();
    println!("  团队分数: {:?}", team_scores);
    
    println!();
}

fn hashset_demo() {
    println!("3. HashSet哈希集合演示:");
    
    // 创建HashSet
    let mut books = HashSet::new();
    books.insert("Rust编程语言");
    books.insert("算法导论");
    books.insert("设计模式");
    
    println!("  书籍集合: {:?}", books);
    
    // 检查是否包含
    if books.contains("Rust编程语言") {
        println!("  找到了Rust编程语言这本书");
    }
    
    // 添加重复元素（不会改变集合）
    let added = books.insert("Rust编程语言");
    println!("  重复添加结果: {}", added);
    println!("  集合大小: {}", books.len());
    
    // 删除元素
    let removed = books.remove("算法导论");
    println!("  删除算法导论: {}", removed);
    println!("  删除后: {:?}", books);
    
    // 集合操作
    let set1: HashSet<i32> = [1, 2, 3, 4, 5].iter().cloned().collect();
    let set2: HashSet<i32> = [4, 5, 6, 7, 8].iter().cloned().collect();
    
    println!("  集合1: {:?}", set1);
    println!("  集合2: {:?}", set2);
    
    // 并集
    let union: HashSet<_> = set1.union(&set2).collect();
    println!("  并集: {:?}", union);
    
    // 交集
    let intersection: HashSet<_> = set1.intersection(&set2).collect();
    println!("  交集: {:?}", intersection);
    
    // 差集
    let difference: HashSet<_> = set1.difference(&set2).collect();
    println!("  差集(set1 - set2): {:?}", difference);
    
    // 对称差集
    let symmetric_diff: HashSet<_> = set1.symmetric_difference(&set2).collect();
    println!("  对称差集: {:?}", symmetric_diff);
    
    // 子集检查
    let subset: HashSet<i32> = [2, 3].iter().cloned().collect();
    println!("  {:?} 是 {:?} 的子集吗? {}", subset, set1, subset.is_subset(&set1));
    
    // 去重示例
    let numbers = vec![1, 2, 2, 3, 3, 3, 4, 4, 4, 4];
    let unique: HashSet<_> = numbers.into_iter().collect();
    let unique_vec: Vec<_> = unique.into_iter().collect();
    println!("  去重后: {:?}", unique_vec);
    
    println!();
}

fn other_collections_demo() {
    println!("4. 其他集合类型演示:");
    
    // BTreeMap - 有序映射
    let mut btree_map = BTreeMap::new();
    btree_map.insert(3, "three");
    btree_map.insert(1, "one");
    btree_map.insert(2, "two");
    btree_map.insert(4, "four");
    
    println!("  BTreeMap (有序): {:?}", btree_map);
    
    // BTreeSet - 有序集合
    let mut btree_set = BTreeSet::new();
    btree_set.insert(3);
    btree_set.insert(1);
    btree_set.insert(4);
    btree_set.insert(2);
    
    println!("  BTreeSet (有序): {:?}", btree_set);
    
    // VecDeque - 双端队列
    let mut deque = VecDeque::new();
    deque.push_back(1);
    deque.push_back(2);
    deque.push_front(0);
    deque.push_front(-1);
    
    println!("  VecDeque: {:?}", deque);
    
    let front = deque.pop_front();
    let back = deque.pop_back();
    println!("  弹出前端: {:?}, 弹出后端: {:?}", front, back);
    println!("  剩余: {:?}", deque);
    
    // LinkedList - 链表
    let mut list = LinkedList::new();
    list.push_back(1);
    list.push_back(2);
    list.push_front(0);
    
    println!("  LinkedList: {:?}", list);
    
    // 字符串集合
    let mut string_vec = vec![
        String::from("apple"),
        String::from("banana"),
        String::from("cherry"),
    ];
    
    string_vec.sort();
    println!("  排序后的字符串: {:?}", string_vec);
    
    // 二维Vector
    let mut matrix = vec![vec![0; 3]; 3]; // 3x3矩阵
    matrix[1][1] = 5;
    matrix[0][2] = 3;
    
    println!("  3x3矩阵:");
    for row in &matrix {
        println!("    {:?}", row);
    }
    
    println!();
}

fn collection_techniques_demo() {
    println!("5. 集合操作技巧演示:");
    
    // 链式操作
    let numbers = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    
    let even_squares: Vec<i32> = numbers
        .iter()
        .filter(|&&x| x % 2 == 0)
        .map(|&x| x * x)
        .collect();
    
    println!("  偶数的平方: {:?}", even_squares);
    
    // 分组操作
    let words = vec!["apple", "banana", "apricot", "blueberry", "cherry"];
    let mut grouped: HashMap<char, Vec<&str>> = HashMap::new();
    
    for word in words {
        let first_char = word.chars().next().unwrap();
        grouped.entry(first_char).or_insert(Vec::new()).push(word);
    }
    
    println!("  按首字母分组:");
    for (letter, words) in &grouped {
        println!("    {}: {:?}", letter, words);
    }
    
    // 查找和搜索
    let students = vec![
        ("Alice", 95),
        ("Bob", 87),
        ("Charlie", 92),
        ("David", 78),
        ("Eve", 88),
    ];
    
    // 查找最高分
    let highest = students.iter().max_by_key(|(_, score)| score);
    if let Some((name, score)) = highest {
        println!("  最高分: {} - {}", name, score);
    }
    
    // 查找及格的学生
    let passed: Vec<_> = students
        .iter()
        .filter(|(_, score)| *score >= 80)
        .collect();
    
    println!("  及格学生:");
    for (name, score) in passed {
        println!("    {} - {}", name, score);
    }
    
    // 计算平均分
    let total: i32 = students.iter().map(|(_, score)| score).sum();
    let average = total as f64 / students.len() as f64;
    println!("  平均分: {:.2}", average);
    
    // 分区操作
    let (passed, failed): (Vec<(&str, i32)>, Vec<(&str, i32)>) = students
        .iter()
        .partition(|(_, score)| *score >= 80);
    
    println!("  及格人数: {}, 不及格人数: {}", passed.len(), failed.len());
    
    println!();
}

fn performance_demo() {
    println!("6. 性能比较演示:");
    
    // Vector vs LinkedList 插入性能
    let mut vec = Vec::new();
    let mut list = LinkedList::new();
    
    // 在末尾插入 - Vector更快
    for i in 0..1000 {
        vec.push(i);
        list.push_back(i);
    }
    
    println!("  Vector和LinkedList都完成了1000次末尾插入");
    
    // HashMap vs BTreeMap 查找性能
    let mut hash_map = HashMap::new();
    let mut btree_map = BTreeMap::new();
    
    for i in 0..1000 {
        hash_map.insert(i, i * 2);
        btree_map.insert(i, i * 2);
    }
    
    println!("  HashMap和BTreeMap都插入了1000个元素");
    
    // 内存使用建议
    println!("  性能建议:");
    println!("    - Vector: 随机访问快，末尾插入删除快");
    println!("    - VecDeque: 两端插入删除快");
    println!("    - LinkedList: 中间插入删除快，但内存开销大");
    println!("    - HashMap: 平均O(1)查找，无序");
    println!("    - BTreeMap: O(log n)查找，有序");
    println!("    - HashSet: 快速去重和集合操作");
    println!("    - BTreeSet: 有序的集合操作");
    
    // 容量预分配示例
    let mut efficient_vec = Vec::with_capacity(1000);
    let mut normal_vec = Vec::new();
    
    // 预分配容量可以避免多次重新分配
    for i in 0..1000 {
        efficient_vec.push(i);
        normal_vec.push(i);
    }
    
    println!("  预分配容量可以提高性能");
    
    println!();
}

// 自定义结构体用于演示
#[derive(Debug, Clone)]
struct Student {
    name: String,
    age: u8,
    grades: Vec<u8>,
}

impl Student {
    fn new(name: &str, age: u8) -> Self {
        Student {
            name: name.to_string(),
            age,
            grades: Vec::new(),
        }
    }
    
    fn add_grade(&mut self, grade: u8) {
        self.grades.push(grade);
    }
    
    fn average_grade(&self) -> f64 {
        if self.grades.is_empty() {
            0.0
        } else {
            self.grades.iter().sum::<u8>() as f64 / self.grades.len() as f64
        }
    }
}

// 演示复杂数据结构
fn complex_data_structures_demo() {
    println!("复杂数据结构演示:");
    
    let mut students = HashMap::new();
    
    let mut alice = Student::new("Alice", 20);
    alice.add_grade(95);
    alice.add_grade(87);
    alice.add_grade(92);
    
    let mut bob = Student::new("Bob", 19);
    bob.add_grade(78);
    bob.add_grade(85);
    bob.add_grade(90);
    
    students.insert("Alice", alice);
    students.insert("Bob", bob);
    
    for (name, student) in &students {
        println!("  {}: 平均分 {:.2}", name, student.average_grade());
    }
    
    // 按平均分排序
    let mut sorted_students: Vec<_> = students.iter().collect();
    sorted_students.sort_by(|a, b| {
        b.1.average_grade().partial_cmp(&a.1.average_grade()).unwrap()
    });
    
    println!("  按平均分排序:");
    for (name, student) in sorted_students {
        println!("    {}: {:.2}", name, student.average_grade());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_vector_operations() {
        let mut vec = vec![1, 2, 3];
        vec.push(4);
        assert_eq!(vec.len(), 4);
        assert_eq!(vec[3], 4);
    }
    
    #[test]
    fn test_hashmap_operations() {
        let mut map = HashMap::new();
        map.insert("key", "value");
        assert_eq!(map.get("key"), Some(&"value"));
        assert_eq!(map.get("nonexistent"), None);
    }
    
    #[test]
    fn test_hashset_operations() {
        let mut set = HashSet::new();
        assert!(set.insert(1));
        assert!(!set.insert(1)); // 重复插入返回false
        assert!(set.contains(&1));
        assert_eq!(set.len(), 1);
    }
    
    #[test]
    fn test_student_average() {
        let mut student = Student::new("Test", 20);
        student.add_grade(80);
        student.add_grade(90);
        assert_eq!(student.average_grade(), 85.0);
    }
}