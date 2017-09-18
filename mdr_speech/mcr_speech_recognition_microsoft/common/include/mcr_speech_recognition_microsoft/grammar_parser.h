#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>

class GrammarParser
{
private:
    std::string grammarFolderName;
    const std::string ruleRefKey; //identifiers for a URL reference in an SAPI grammar
    const std::string urlKey; //identifiers for the URL in an SAPI grammar

    bool identifyGrammarsToCombine(std::vector<std::string>& listOfReferencedGrammars, std::string grammarToParseRelativePath);
    std::string extractURLFileName(std::string dataString);
    std::string combineGrammarFiles(std::vector<std::string> files);

public:
    GrammarParser();
    GrammarParser(std::string grammarFolderName);
    void setGrammarFolder(std::string grammarFolder);

    bool parseGrammarsToOneFile(std::string grammarToBeParsedFileName, std::string nameForTemporaryGrammar);
};