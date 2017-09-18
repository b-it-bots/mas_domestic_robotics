#include "mcr_speech_recognition_microsoft/string_operations.h"

std::string StringOperations::trim(std::string stringToTrim)
{
    std::string result;

    std::stringstream trimmer;
    trimmer << stringToTrim;
    trimmer >> result;

    return result;
}

/**
* Convert an std::string to an LPW string
*/
std::wstring StringOperations::stdstring2LPW(const std::string& s)
{
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}

/**
* convert an LPCW string to an std::string
*/
std::string StringOperations::LPCW2stdstring(const LPCWSTR s)
{
    std::wstring data(s);
    // Kopiere in std::vector inklusive Nullterminierung
    std::vector<wchar_t> vec(data.begin(), data.end());
    vec.push_back(L'\0');

    return LPW2stdstring(&vec[0]);
}

/**
* Convert an LPW string to an std::string
*/
std::string StringOperations::LPW2stdstring(LPWSTR pw, UINT codepage)
{
    std::string s;
    char* p = 0;
    int bsz;

    bsz = WideCharToMultiByte(codepage,
                              0,
                              pw, -1,
                              0, 0,
                              0, 0);
    if (bsz > 0)
    {
        p = new char[bsz];

        int rc = WideCharToMultiByte(codepage,
                                     0,
                                     pw, -1,
                                     p, bsz,
                                     0, 0);

        if (rc != 0)
        {
            p[bsz - 1] = 0;
            s = p;
            // res = true;
        }
    }
    delete [] p;
    //return res;
    return s;
}

/**
* test if a string vector contains  a specific string
*
* @param vector: elemnts to search through
* @param elementToCheck: string to check if it is contained
* @return: true if elementToCheck is in vector
*/
bool StringOperations::isContained(std::vector<std::string> vector, std::string elementToCheck)
{
    for (unsigned int i = 0; i < vector.size(); i++)
    {
        if (vector[i] == elementToCheck)
        {
            return true;
        }
    }
    return false;
}

size_t StringOperations::caseInsensitiveFind(const std::string& stringToFind, const std::string& stringToSearchIn)
{
    std::string stringToFindCopy(stringToFind);
    std::string stringToSearchInCopy(stringToSearchIn);
    std::transform(stringToFindCopy.begin(), stringToFindCopy.end(), stringToFindCopy.begin(), ::toupper);
    std::transform(stringToSearchInCopy.begin(), stringToSearchInCopy.end(), stringToSearchInCopy.begin(), ::toupper);
    return stringToSearchInCopy.find(stringToFindCopy);
}

std::string& StringOperations::replaceAll(std::string& context, const std::string& from, const std::string& to)
{
    size_t lookHere = 0;
    size_t foundHere;
    while ((foundHere = context.find(from, lookHere)) != std::string::npos)
    {
        context.replace(foundHere, from.size(), to);
        lookHere = foundHere + to.size();
    }
    return context;
}