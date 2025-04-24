#projects #biped-drone #notes
Foundation of many modern technologies. It is a multi paradigm programming language that supports generic programming and low-level manipulation, function overloading, and the four primary features of *Object Oriented Programming*:
- Encapsulation.
- Polymorphism.
- Abstraction.
- Inheritance.
## Features 
- *Simple*: Program can be divided into logical parts. Rich library support.
- *Machine Independent*: Code can be run anywhere as long as compiler installed.
- *Low-Level Access*: Suitable for efficient system coding due to low-level access to system resources.
- *Fast-Execution Speed*: One of the fastest programming languages.
- *Object-Oriented*: Suitable for extensible and maintainable programs.
## Applications
- *Real-Time Systems*
	- Real time embedding for critical applications.
	- Low-latency performance in embedded systems.
- *Gaming Engines*
	- Real time rendering.
	- High performance.
- *Application Software*
	- Fast execution and interaction.
	- Usage for complex tasks.
- *Operating Systems*
	- C++ based GUI's for performance and stability.
	- Used in system components for hardware interaction.
- *Embedded Systems*
	- Micro-controller platform for hardware programming.
	- Hardware control in Robotics and IoT.
## Characteristics
- *Type*: Compiled
- *Paradigm*: Multi-paradigm (procedural, object-oriented, generic).
- *Memory Management*: Manual.
- *Syntax*: Complex.
- *Use Cases*: System programming, game development, high-performance applications.
- *Notable Frameworks & Libraries*: Standard Library, Boost.
- *Community Support*: High.
- *Job Market*: Abundant.
## Features of C++
- *Object-Oriented Programming*: It can create/destroy objects while programming. 
	- *Concepts*: Class, Objects, Encapsulation, Polymorphism, Inheritance, Abstraction.
- *Machine-Independent*: The code can run in any OS.This does not apply to the executable.
- *Simple*: Can be broken down into logical units and parts.
	- *Auto-Keyword*: To deduce the type of data automatically.
- *High-Level Language*: Closely associated with human-comprehensible language.
- *Popular*: Can be the base language of many other object-oriented programming languages.
- *Case-Sensitive*: Take care of upper-lower case usage in the program.
- *Compiler-Based*: The program should be compiled into an executable, which is then run.
- *Dynamic Memory Allocation*: When running, in the *dynamical heap space*; inside functions, in the *stack space*. Required memory can be determined at run time.
- *Memory Management*: 
	- Variable/array memory allocation during run-time, known as *Dynamic Memory Allocation*.
	- The compiler doesn't automatically manage the memories allocated for the variables.
	- Allocation and deallocation must be done manually, through the *new* and *delete* operators respectively.
- *Multi-Threading*: Contains two or more parts that will run concurrently. Each part is named a *thread* and every thread defines a different execution path.
	- Specialized form of multi-tasking. To execute two or more programs concurrently. Two sorts:
		- *Process-based*: Handles the concurrent execution of programs.
		- *Thread-based*: Deals with the multi programming of pieces of an equivalent program.
	- Easy to work with multi threads via *std::thread*.
## Literals 
There are four types of literals in C++.
### Integer Literal 
- *Decimal-literal (base 10)*: non-zero decimal digit lead by more decimal digits.
- *Octal-literal (base 8)*: a **0** followed by zero or more octal digits.
- *Hex-literal (base 16)*: **0x** or **0X** followed by one or more hexadecimal digits(0~9, a~f).
- *Binary-literal (base 2)*: **0b** or **0B** followed by one or more binary digits.
- *Suffixes*: indicate the type in which it is to be read.
	- *int*: default data type, no suffix needed.
	- *long int*: l or L at the end of the integer.
	- *unsigned int*: ul or UL at the end of the integer.
	- *long long int*: ll or LL at the end of the integer.
	- *unsigned long long*: ull or ULL at the end of the integer.
### Float Literal
To represent and store real numbers. They are composed of four parts, the **integer part, real part, fractional part,** and **exponential part**. 
**Examples:**
- `10.125`
- `1.215e-10L`
- `10.5E-3`

