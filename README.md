# A* Pathfinding Implementation in Rust, Python, and C++
### **Project members**
    *  Hardaat Singh Baath       -   2021A7PS2662G 
    *  Sehajpreet Singh Chawla   -   2021A7PS2048G
    *  Manraj Singh Chahal       -   2021A7PS2630G
    *  Vishesh Goyal             -   2021A7PS2628G

## Problem Statement
### Original Statement:
    The project aims to implement the A* pathfinding algorithm in Rust, Python, and C++. The primary focus is on optimizing execution time, CPU utilization, and dynamic memory allocation, particularly in the context of a 2D matrix representing an occupancy grid.

### POPL Angle:
    The core of the problem lies in efficiently finding the optimal path in a grid, a common challenge in robotics, gaming, and various AI applications. The POPL angle here involves optimizing the code for performance, memory usage, and considering language-specific nuances. While A* is a well-known algorithm, the unique aspect is the emphasis on language-specific optimizations and their impact on the mentioned metrics.

### Solution Differentiation:
    While A* implementations exist, our solution differentiates itself by focusing on language-specific optimizations, particularly in Rust, Python, and C++. The project explores how each language's features and characteristics influence the performance of the algorithm. In terms of papers on algorithm comparison, we weren't able to find any previous papers on it, and we have done our own implementation.

## Software Architecture
### Overview:
    The project follows a modular architecture with language-specific implementations of the A* algorithm. Each implementation includes modules for grid representation, A* logic, and performance testing.

### Reusability:
    We went throught the implementation of A* algorithm and wrote it in Python. We then converted the same code in C++ as well. Common algorithmic components are shared across implementations, emphasizing code modularity. The implementations differ in how they handle memory, threading, and other language-specific features. 

### Testing Component:
    The testing component is integrated into each implementation. Performance tests measure execution time, CPU utilization, and memory usage. The testing component can run locally or remotely, depending on the chosen configuration. For the values calculated, we took averages to ensure that outliers do not effect our data.

### Database:
    No database is involved in this project as the focus is on algorithmic efficiency and language-specific optimizations.

## Languages used  for the Project

### Rust:
* **Memory Safety:**
Rust's ownership system ensures memory safety without the need for garbage collection. This can help prevent common memory-related errors.
* **Performance:**
Rust is designed for performance. It provides low-level control over system resources without sacrificing safety.
* **Concurrency:**
Rust's ownership system also helps prevent data races, making concurrent programming more manageable and safer.
* **Expressive Type System:**
Rust's type system allows for expressive and flexible code, enabling you to write concise and clear implementations.

### C++:
* **Performance:**
C++ is known for its performance and efficiency. It provides low-level control and allows for fine-tuning of algorithms.
* **Standard Template Library (STL):**
The STL in C++ provides a rich set of data structures and algorithms, making it easier to implement A* with standard containers.
* **Object-Oriented Programming (OOP):**
C++ supports OOP principles, which can be beneficial for organizing code and creating reusable components.
* **Legacy Code Compatibility:**
C++ has been widely used for a long time, and there are many libraries and existing codebases that can be leveraged.

### Python:
* **Ease of Implementation:**
Python is known for its readability and simplicity, making it easier to implement algorithms quickly.
* **Rapid Prototyping:**
Python's dynamic typing and high-level abstractions allow for rapid prototyping, making it suitable for experimentation.
* **Community and Libraries:**
Python has a large and active community with a rich ecosystem of libraries. Implementing A* might be easier with existing data structures and utility libraries.
* **Interpreted Language:**
Python's interpreted nature allows for quick testing and debugging, facilitating the development process.

#### Considerations:
* **Algorithm Complexity:**
If performance is critical and the algorithm will be applied to large datasets, Rust or C++ might be more suitable due to their emphasis on low-level control and efficiency.
* **Prototyping and Experimentation:**
For quick prototyping and experimentation, Python's simplicity and high-level abstractions might be advantageous.
* **Existing Codebase:**
Consider the existing codebase or libraries you may want to leverage. The language that integrates well with existing components could be a practical choice.

In summary, the choice between Rust, C++, and Python depends on factors such as performance requirements, ease of implementation, and the specific goals of your project. Each language has its strengths, and the decision should be based on a careful evaluation of these factors in the context of your application.


## Principles of Programming Language used
### Ownership and Lifetimes:

* **Definition:** Ownership is Rust's mechanism for managing memory and ensuring memory safety without a garbage collector. It involves tracking which part of code owns a piece of data and is responsible for cleaning it up.

* **How it Works:** Each value in Rust has a variable that is its "owner." A value can only have one owner at a time. When the owner goes out of scope, Rust automatically cleans up the memory associated with the value.
* **Use in code:**
```
Ownership and lifetimes are implicit in the usage of references
```

### Referencing and Borrowing:

* **Definition:** 
    * Borrowing: is the mechanism by which references to values are passed around in Rust without transferring ownership.
Borrowing allows multiple parts of the code to read or use data without taking ownership.

    * Referencing:  Referencing is the act of creating references to values, allowing functions or other parts of the code to operate on the data without taking ownership.
* **Types of Borrowing:**
    1. **Immutable Borrowing (&T):** Allows reading the data but not modifying it.
    2. **Mutable Borrowing (&mut T):** Allows both reading and modifying the data but ensures exclusive access.
* **Use in code:**
```
Line 50: In the astar function, grid is passed as a reference to a slice of slices: &[Vec<i32>]. This is borrowing the grid without taking ownership.
Line 98: In the get_neighbors function, node is borrowed when calling get_neighbors(current, width, height):
Line 128: In the heuristic function, start and goal are borrowed when calculating the Manhattan distance:
```

### Mutability:

* **Definition:** Rust distinguishes between mutable and immutable references. It enforces the principle that either one or the other can occur, preventing data races at compile time.
* **How it Works:** When a reference is mutable (&mut T), it allows changes to the underlying data. However, there can be only one mutable reference to a piece of data in a given scope to avoid data races.
* **Use in code:**
```
[Line 4](C:\Users\vishe\Desktop\POPL_GROUP_14\code-orig\rust_code.rs#L4)
Line 6
Line 8
Line 9
Line 11
```

### Pattern Matching:

* **Definition:** Pattern matching is a programming language feature that allows you to match complex data structures, such as algebraic data types or data structures with nested components, against a specific pattern. It is a way of expressing conditional behavior based on the shape or structure of data.
* **Use in code:**
```
Line 135: Pattern matching in the while loop
```

### Traits:

* **Definition:** Traits define a set of methods that can be implemented by types. They are a way to group method signatures together and provide a common interface for different types.
* **Purpose:** Traits enable code reuse and polymorphism by allowing multiple types to implement the same set of methods
* **Use in code:**
```
Line 30: Ord trait implementation for Node
Line 38: PartialOrd trait implementation for Node
```

## Results

## Scope of Future Work
1. **GPU Acceleration:** 
    * Explore GPU parallelization for further performance improvements.
2. **Optimizations for Large-scale Grids:** 
    * Investigate optimizations specifically tailored for extremely large occupancy grids.
3. **Integration with Real-world Applications:**
    * Implement A* as part of a larger system, considering real-world use cases and constraints.
4. **Broader Evaluation base:**
    * Incorporation of additional languages for a broader comparison. 
    * Investigation of language-specific optimizations and best practices for A* implementation.
5. **Diverse Hardware Testing and Threading:**
    * Testing on diverse hardware and platforms to evaluate platform-specific variations.
    * Exploration of multi-threading and parallelization for performance optimization.