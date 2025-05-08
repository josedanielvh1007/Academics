---
tags:
  - projects
  - biped-drone
  - notes
topic: modern C++
date_created: 
date_modified: 
---

___
## Basic Print
~~~cpp
#include <iostream>
using namespace std;
int main(){
	cout << "Hello World!" << endl;
	return 0;
}
~~~
## Data Receiving
~~~cpp
#include <iostream>
using namespace std;
int main() {
	int age;
	cout << "Enter your age: ";
	cin >> age;
	cout << "Age entered: " << age << endl;
	cout << "Size of int: " << sizeof(int) << " bytes" << endl;
	cout << "Size of char: " << sizeof(char) << " bytes" << endl;
	cout << "Size of float: " << sizeof(float) << " bytes" << endl;
	cout << "Size of double: " << sizeof(double) << " bytes" << endl;
	return 0;
}
~~~
**Note**: Data types can be modified. Four types of modifiers in C++: **short, long, signed** and **unsigned**. 