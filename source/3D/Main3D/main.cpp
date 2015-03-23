#include <iostream>
#include <string>

using namespace std;

extern void func();

void main()
{
	func();
	cout << "Done" << endl;

#ifdef _DEBUG
	cout << "Done, press ENTER to finish" << endl;
	string s;
	getline(cin, s);
#endif
}