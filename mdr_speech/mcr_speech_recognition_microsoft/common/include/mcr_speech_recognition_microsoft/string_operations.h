#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <windows.h>
#include <sstream>

class StringOperations
{
public:
    static std::wstring stdstring2LPW(const std::string& s);
    static std::string LPW2stdstring(LPWSTR pw, UINT codepage = CP_ACP);
    static std::string LPCW2stdstring(const LPCWSTR s);
    static bool StringOperations::isContained(std::vector<std::string> vector, std::string elementToCheck);
    static size_t caseInsensitiveFind(const std::string& stringToFind, const std::string& stringToSearchIn);
    static std::string trim(std::string stringToTrim);
    static std::string& replaceAll(std::string& context, const std::string& from, const std::string& to);
};