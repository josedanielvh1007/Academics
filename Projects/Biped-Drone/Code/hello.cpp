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
	cout << "Size of double: " << sizeof(double) << "bytes" << endl;
	return 0;
}
