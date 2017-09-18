#include "mcr_speech_recognition_microsoft/config_file_reader.h"
#include "mcr_speech_recognition_microsoft/string_operations.h"

ConfigFileReader::ConfigFileReader()
{
    this->configFile = "config.cfg";
    ROSMASTER_CONFIG_FILE_ID = "ROSMASTER:";
    LOCALIP_CONFIG_FILE_ID = "LOCALIP:";
    GRAMMAR_CONFIG_FILE_ID = "DEFAULTGRAMMAR:";
    MEDIA_FOLDER_CONFIG_FILE_ID = "MEDIAFOLDER:";
    GRAMMAR_FOLDER_CONFIG_FILE_ID = "GRAMMARFOLDER:";
    ACKNOWLEDGE_FILE_CONFIG_FILE_ID = "ACKNOLWDGESOUND:";
}

std::string ConfigFileReader::getROSMaster()
{
    return searchForKeyword(ROSMASTER_CONFIG_FILE_ID);
}

std::string ConfigFileReader::getGrammar()
{
    return searchForKeyword(GRAMMAR_CONFIG_FILE_ID);
}


std::string ConfigFileReader::getLocalIP()
{
    return searchForKeyword(LOCALIP_CONFIG_FILE_ID);
}

std::string ConfigFileReader::getMediaFolder()
{
    return searchForKeyword(MEDIA_FOLDER_CONFIG_FILE_ID);
}

std::string ConfigFileReader::getAcknowledgeSoundFile()
{
    return searchForKeyword(ACKNOWLEDGE_FILE_CONFIG_FILE_ID);
}

std::string ConfigFileReader::getGrammarFolder()
{
    return searchForKeyword(GRAMMAR_FOLDER_CONFIG_FILE_ID);
}

void ConfigFileReader::setConfigFile(std::string configFile)
{
    this->configFile = configFile;
}

bool ConfigFileReader::isFileExisting()
{
    if (FILE * file = fopen(this->configFile.c_str(), "r"))
    {
        fclose(file);
        return true;
    }
    return false;
}

/**
* reads the config file and searches for a keyword, returns the rest of the line after the keyword
*
* @return -1 if keyword not found or configfile not there
*/
std::string ConfigFileReader::searchForKeyword(std::string keyword)
{
    std::string result;
    std::ifstream configFile(this->configFile.c_str());
    if (configFile.is_open())
    {
        while (configFile.good())
        {
            std::string line;
            std::getline(configFile, line);
            //store ROSMASTER
            if (line.find(keyword) != std::string::npos)
            {
                result = StringOperations::trim(line.substr((line.find(keyword) + keyword.length())));
            }
        }
        configFile.close();

        if (result.empty())
        {
            //add keyword not found exception
            result = -1;
        }
    }
    else
    {
        //add file not found exception
        result = -1;
        std::cout << "config_file could not be opened.";
    }
    return result;
}

