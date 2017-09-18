#include "mcr_speech_recognition_microsoft/grammar_parser.h"
#include "mcr_speech_recognition_microsoft/string_operations.h"

GrammarParser::GrammarParser() : ruleRefKey("RULEREF"), urlKey("URL")
{
    this->grammarFolderName = "./grammars/";
}

GrammarParser::GrammarParser(std::string grammarFolderName) : ruleRefKey("RULEREF"), urlKey("URL")
{
    this->grammarFolderName = grammarFolderName;
}

void GrammarParser::setGrammarFolder(std::string grammarFolder)
{
    this->grammarFolderName = grammarFolder;
}


/**
* Parse a grammar to find ruleref tags. Return a list of all referenced files.
* Recursively check all referenced files
* This is necessary since the SAPI does not correcly parse grammars that reference to other files
*
* @param listOfReferencesGrammars: returns all grammars that are referenced
* @param grammarToParse: The grammar that shall be checked for references
* @return false if grammar file could not be opened
*/
bool GrammarParser::identifyGrammarsToCombine(std::vector<std::string>& listOfReferencedGrammars, std::string grammarToParseRelativePath)
{
    std::string line;
    std::ifstream grammarFile(grammarToParseRelativePath.c_str());

    if (grammarFile.is_open())
    {
        while (grammarFile.good())
        {
            getline(grammarFile, line);

            //check if line contains a ruleref
            if (StringOperations::caseInsensitiveFind(ruleRefKey, line) != std::string::npos)
            {
                std::string grammarURL = extractURLFileName(line);

                //Add file only if not already inserted (or is empty)
                if (!grammarURL.empty() && !StringOperations::isContained(listOfReferencedGrammars, grammarURL))
                {
                    listOfReferencedGrammars.push_back(grammarURL);

                    //parse identified grammar, return false on error
                    if (!identifyGrammarsToCombine(listOfReferencedGrammars, grammarFolderName + grammarURL))
                    {
                        return false;
                    }
                }
            }
        }
        grammarFile.close();
    }
    else
    {
        std::cout << "Unable to open file: " << grammarToParseRelativePath;
        return false;
    }
    return true;
}


/**
* search the first occurance of the term url or URL in a string and
* returns the following term that is enclosed in quotation marks
*
* @parm dataString: String that shall be searched
* @return: in quotationmarks enclosed string after URL
*/
std::string GrammarParser::extractURLFileName(std::string dataString)
{
    //index of the url
    size_t urlFoundAt = std::string::npos;

    urlFoundAt = StringOperations::caseInsensitiveFind(urlKey, dataString);

    if (urlFoundAt != std::string::npos) //url was found!
    {
        //from the url position find opening and closing quotations to identify url position and extract the string
        size_t subGrammarFileNameBegin = dataString.find("\"", urlFoundAt);
        subGrammarFileNameBegin = subGrammarFileNameBegin + 1;
        size_t subGrammarFileNameEnd = dataString.find("\"", subGrammarFileNameBegin + 1);
        std::string result = dataString.substr(subGrammarFileNameBegin, subGrammarFileNameEnd - subGrammarFileNameBegin);
        return result;
    }
    else
    {
        return "";
    }
}


/**
* Combine the passeed grammar files which locations are stored in the vector, startimg with the
* single file in the first parameter
*
* @param: files: list of grammar files that shall be combined
* @return: string containing the combined grammar
*/
std::string GrammarParser::combineGrammarFiles(std::vector<std::string> files)
{
    std::string combinedGrammar;
    combinedGrammar.append("<grammar LANGID=\"409\">");
    combinedGrammar.append("\n");

    //Parse every grammar file
    for (unsigned int i = 0; i < files.size(); i ++)
    {
        //add mediafolder
        std::string fileToParse = grammarFolderName;
        fileToParse.append(files[i]);

        std::ifstream grammarFile(fileToParse.c_str());
        std::string line;
        if (grammarFile.is_open())
        {
            while (grammarFile.good())
            {
                getline(grammarFile, line);


                //ignore grammar tags, they are manually added at the beginning / end of the temp file
                if (StringOperations::caseInsensitiveFind("<grammar", line) == std::string::npos
                        && StringOperations::caseInsensitiveFind("</grammar>", line) == std::string::npos)
                {
                    //remove the url in the ruleref
                    if (StringOperations::caseInsensitiveFind(ruleRefKey, line) != std::string::npos)
                    {
                        size_t urlIndex = StringOperations::caseInsensitiveFind(urlKey, line);

                        //if URL/url is found, extract the definition between url and closing quotation
                        if (urlIndex != std::string::npos)
                        {
                            combinedGrammar.append(line.substr(0, urlIndex));
                            //find end of URL definition (closing quotation
                            size_t urlEnd = line.find("\"", urlIndex);
                            urlEnd = line.find("\"", urlEnd + 1);
                            //append everything after
                            combinedGrammar.append(line.substr(urlEnd + 1));
                        }
                    }
                    else
                    {
                        //write line to the new temp grammar
                        combinedGrammar.append(line);
                        combinedGrammar.append("\n");
                    }
                }
            }
            grammarFile.close();
        }
    }
    combinedGrammar.append("</grammar>");

    return combinedGrammar;
}


/**
* Parses a grammar (located in specified grammar folder) and writes it and all URL referenced grammars in a single file specified in the paramter
* this is necessary because the SAPI grammar compiler does not resolve file references
*
* @param: grammarToBeParsedFileName: grammar file that shall be parsed
* @param: nameForTemporaryGrammar: file where the combined grammars shall be stored
*/
bool GrammarParser::parseGrammarsToOneFile(std::string grammarToBeParsedFileName, std::string nameForTemporaryGrammar)
{
    bool isParsingSuccessfull = false;
    std::vector<std::string> files;
    //add the main file initially to the ones that shall be combined
    files.push_back(grammarToBeParsedFileName);

    isParsingSuccessfull  = identifyGrammarsToCombine(files, grammarFolderName + grammarToBeParsedFileName);

    //if successfull: write to file
    if (isParsingSuccessfull)
    {
        std::string tempCombinedGrammar = combineGrammarFiles(files);
        std::ofstream tempGrammar(nameForTemporaryGrammar.c_str());
        if (tempGrammar.is_open())
        {
            tempGrammar << tempCombinedGrammar;
        }
    }
    return isParsingSuccessfull;
}