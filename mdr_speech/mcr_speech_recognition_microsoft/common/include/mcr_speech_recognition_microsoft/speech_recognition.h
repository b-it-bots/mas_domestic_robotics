#include <windows.h>
#include <sphelper.h> //indludes speech relevant API
#include <string>

#include "mcr_speech_recognition_microsoft/grammar_parser.h"

class SpeechRecognition
{
private:
    /*************Members***************/

    //Folder and files definitions
    std::string mediaFolderName;
    std::string acknowledgeSoundFileName;
    std::string grammarFolderName;

    float confidenceThreshold;

    //Speech recognition data
    CComPtr<ISpRecoGrammar> cpGram; //smart pointer
    HANDLE hEvent;  //Evvent, occurs when recognizer has done recognizing
    CSpEvent ect;
    CComPtr<ISpRecoContext> cpRecoCtx; //smart pointer
    LPWSTR pwszText; //Text received from recognizer
    SPPHRASE *pParts;
    ISpPhrase *pPhrase;
    ULONGLONG ullEvents; //EventMask
    CComPtr<ISpRecognizer> cpEngine; //smart pointer
    CComPtr<ISpAudio> cpAudio; //Audio Device
    CComPtr<ISpObjectToken> cpObjectToken; // Audio input token

    /*******************Methods*****************/


public:
    const std::string noSpeechKeyword;
    const std::string notUnderstoodKeyword;

    SpeechRecognition();
    SpeechRecognition(float confidenceThreshold, std::string mediaFolderName, std::string grammarFolderName,
                      std::string acknowledgeSoundFileName);
    ~SpeechRecognition();
    float getConfidenceThreshold();
    void setConfidenceThreshold(float threshold);

    void setMediaFolderName(std::string mediaFolderName);
    void setGrammarFolderName(std::string mediaFolderName);
    void setAcknowledgeSoundFileName(std::string acknowledgeSoundFileName);

    void DecomposeSpeech(const SPPHRASEPROPERTY* data, std::vector<std::string>* keywords, std::vector<float>* confdences);
    HRESULT loadGrammar(std::string grammarFile);   //Load a grammar defined in passed files
    void CheckForPhrase(std::string* understoodPhrase, std::string* matchedKeyword, float* resultConfidence, std::vector<std::string>* resultKeywordList, std::vector<float>* resultConfidenceList);
    HRESULT initSpeech(std::string grammarFileName);

};