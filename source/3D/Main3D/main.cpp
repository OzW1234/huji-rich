#include <iostream>
#include <string>

using namespace std;


void main()
{
	cout << "Done" << endl;

#ifdef _DEBUG
	cout << "Done, press ENTER to finish" << endl;
	string s;
	getline(cin, s);
#endif
}