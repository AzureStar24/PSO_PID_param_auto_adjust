#ifndef KEY_VALUE_H
#define KEY_VALUE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <string>
#include <vector>
#include <cstdio>

using namespace std;

string getKeyValue(const string& filename, const string& title, const string& key);
void setKeyValue(const string& filename, const string& title, const string& key, const string& value);



#endif