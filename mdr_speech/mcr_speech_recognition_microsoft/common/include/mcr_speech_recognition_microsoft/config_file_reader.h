#include <string>
#include <fstream>

class ConfigFileReader
{
public:
    ConfigFileReader();

    std::string getROSMaster();
    std::string getLocalIP();
    std::string getGrammar();
    std::string getMediaFolder();
    std::string getAcknowledgeSoundFile();
    std::string getGrammarFolder();
    bool isFileExisting();

    void setConfigFile(std::string configFile);
    std::string searchForKeyword(std::string keyword);

private:
    std::string ROSMASTER_CONFIG_FILE_ID;
    std::string LOCALIP_CONFIG_FILE_ID;
    std::string GRAMMAR_CONFIG_FILE_ID;
    std::string MEDIA_FOLDER_CONFIG_FILE_ID;
    std::string GRAMMAR_FOLDER_CONFIG_FILE_ID;
    std::string ACKNOWLEDGE_FILE_CONFIG_FILE_ID;

    std::string configFile;

};