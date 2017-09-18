/************************************************
* Microsoft speech recognition (SAPI) from Tutorial:
* http://msdn.microsoft.com/en-us/visualc/Video/cc482921
*
* Author: Mike
* Created: 2011-02-07
* Modified:
*************************************************/
#include <iostream>
#include <sphelper.h> //indludes speech relevant API

// for playing sound
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <vector>

#include "mcr_speech_recognition_microsoft/speech_recognition.h"
#include "mcr_speech_recognition_microsoft/string_operations.h"

SpeechRecognition::SpeechRecognition() : noSpeechKeyword("no_speech"), notUnderstoodKeyword("not_understood")
{
    this->confidenceThreshold = 0.6f;
    this->mediaFolderName = "./media/";
    this->grammarFolderName = "/grammars/";
    this->acknowledgeSoundFileName = "beep1.wav";

}

SpeechRecognition::SpeechRecognition(float confidenceThreshold, std::string mediaFolderName,
                                     std::string grammarFolderName, std::string acknowledgeSoundFileName)
    : noSpeechKeyword("no_speech"), notUnderstoodKeyword("not_understood")
{
    this->confidenceThreshold = confidenceThreshold;
    this->mediaFolderName = mediaFolderName;
    this->grammarFolderName = grammarFolderName;
    this->acknowledgeSoundFileName = acknowledgeSoundFileName;

}

void SpeechRecognition::setConfidenceThreshold(float threshold)
{
    if (threshold > 1.0f)
    {
        this->confidenceThreshold = 1.0f;
    }
    else if (threshold < 0.0f)
    {
        this->confidenceThreshold = 0.0f;
    }
    else
    {
        this->confidenceThreshold = threshold;
    }
}

float SpeechRecognition::getConfidenceThreshold()
{
    return confidenceThreshold;
}

void SpeechRecognition::setMediaFolderName(std::string mediaFolderName)
{
    this->mediaFolderName = mediaFolderName;
}

void SpeechRecognition::setGrammarFolderName(std::string grammarFolderName)
{
    this->grammarFolderName = grammarFolderName;
}

void SpeechRecognition::setAcknowledgeSoundFileName(std::string acknowledgeSoundFileName)
{
    this->acknowledgeSoundFileName = acknowledgeSoundFileName ;
}

/**
* Initialize all speech relevant elements
*
* @return: Error code HRESULT
*/
HRESULT SpeechRecognition::initSpeech(std::string grammarFileName)
{
    CoInitialize(0); //Initialize COM - Application
    HRESULT errorcode;

    // Get the default audio input token (Microphone).
    errorcode = SpGetDefaultTokenFromCategoryId(SPCAT_AUDIOIN, &cpObjectToken);
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Could not get default audio input token" << std::endl;
        return errorcode;
    }

    //Initialize recognizer
    errorcode = cpEngine.CoCreateInstance(CLSID_SpInprocRecognizer); //InProcess = Only for this process microphone is available but we have to initialize audio device
    if (FAILED(errorcode))                                          //SharedRecognizer = Use Microphone set in system settings and share with all processes (windows commands active!)
    {
        std::wcout << "HRESULT: " << errorcode << " Could not create speech recognizer" << std::endl;
        return errorcode;
    }

    // Set the audio input to our token
    errorcode = cpEngine->SetInput(cpObjectToken, TRUE);
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Could not set audio input for speech recognizer" << std::endl;
        return errorcode;
    }

    //Create the speech recognition context
    errorcode = cpEngine->CreateRecoContext(&cpRecoCtx);
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Could not create the speech recognition context" << std::endl;
        return errorcode;
    }

    errorcode = cpRecoCtx->SetNotifyWin32Event(); // Set speech API to use w32 events, SAPI generates events and we only have to wait
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Could not register event listener on speech context" << std::endl;
        return errorcode;
    }

    //get the event handler
    hEvent = cpRecoCtx->GetNotifyEventHandle();

    //Many different events are possible (hear music, hear speech, got hypothesis... Have to look into that!
    ullEvents = SPFEI(SPEI_RECOGNITION) | SPFEI(SPEI_FALSE_RECOGNITION); //event when speechRecognition or FalseRecognition
    errorcode = cpRecoCtx->SetInterest(ullEvents, ullEvents); //Parameters: (events to be Notified, Events to go on the Queue)
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Could not register events to be notified on" << std::endl;
        return errorcode;
    }

    errorcode = cpRecoCtx->CreateGrammar(1, &cpGram); //Parameters: Identifier, result
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Could not create grammar for the context" << std::endl;
        return errorcode;
    }

    errorcode = loadGrammar(grammarFileName);
    if (FAILED(errorcode))
    {
        std::wcout << "HRESULT: " << errorcode << " Error loading grammar: " << grammarFileName.c_str() << std::endl;
        return errorcode;
    }

    if (FAILED(errorcode))
    {
        std::wcout << "check: http://www.nextup.com/sapi5doc/Error_Codes.htm for resolving error codes" << std::endl;
    }

    return errorcode;
}



/**
* load a grammar defined in the passed file. searches in the grammars folder
*
* @Param grammarFile file containing grammar
* @Result HRESULT if error occured
*/
HRESULT SpeechRecognition::loadGrammar(std::string grammarFileName)
{
    HRESULT errorcode;      //used for return codes
    GrammarParser parser;
    parser.setGrammarFolder(this->grammarFolderName);

    std::string combinedGrammarRelativePath = grammarFolderName + "tempGrammar.xml";
    parser.parseGrammarsToOneFile(grammarFileName, combinedGrammarRelativePath);

    std::wstring stemp = StringOperations::stdstring2LPW(combinedGrammarRelativePath);
    LPCWSTR grammarFileRelativePathLPCWSTR = stemp.c_str();

    //Load a Grammar from file
    errorcode = cpGram->LoadCmdFromFile(grammarFileRelativePathLPCWSTR, SPLO_STATIC);
    if (SUCCEEDED(errorcode))
    {
        std::cout << "Loaded Grammar=" << grammarFileName << std::endl;
    }

    // define all rules are active
    errorcode = cpGram->SetRuleState(0, 0, SPRS_ACTIVE);

    std::string understoodPhrase;
    std::string matchedKeyword;
    float resultConfidence;
    std::vector<std::string> resultKeywordList;
    std::vector<float> resultConfidenceList;
    this->CheckForPhrase(&understoodPhrase, &matchedKeyword, &resultConfidence, &resultKeywordList, &resultConfidenceList);

    return errorcode;
}

/**
* Main Speech recognition loop that checks if a new phrase was understood
*
* @param
*/
void SpeechRecognition::CheckForPhrase(std::string* understoodPhrase, std::string* matchedKeyword, float* resultConfidence, std::vector<std::string>* resultKeywordList, std::vector<float>* resultConfidenceList)
{
    *resultConfidence = -1;
    HRESULT errorcode;
    //wait for hEvent max 0s
    WaitForSingleObject(hEvent, 0);

    //If we got speech and no time out
    if (ect.GetFrom(cpRecoCtx) == S_OK)
    {
        //if phrase matched grammar
        if (ect.eEventId == SPEI_RECOGNITION)
        {
            pPhrase = ect.RecoResult();
            errorcode = pPhrase->GetPhrase(&pParts);
            errorcode = pPhrase->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, FALSE, &pwszText, 0);

            *understoodPhrase = StringOperations::LPW2stdstring(pwszText);

            //With rule refs in the grammar, the value of a rule resp. node can be NULL, we override these elements... awful but works
            while (pParts->pProperties->pszValue == NULL && pParts->pProperties->pFirstChild != NULL)
            {
                pParts->pProperties = pParts->pProperties->pFirstChild;
            }

            if (pParts->pProperties->pszValue != NULL && pParts->Rule.SREngineConfidence >= confidenceThreshold)
            {
                PlaySound((mediaFolderName + acknowledgeSoundFileName).c_str(), NULL, SND_FILENAME | SND_SYNC);
                *resultConfidence = pParts->Rule.SREngineConfidence;
                *matchedKeyword = StringOperations::LPCW2stdstring(pParts->pProperties->pszValue);
            }

            resultConfidenceList->clear();
            resultKeywordList->clear();
            //Decompose all keywords an their confidence
            this->DecomposeSpeech(pParts->pProperties, resultKeywordList, resultConfidenceList);


            //clear the not needed memory
            CoTaskMemFree(pParts);
            CoTaskMemFree(pwszText);
        }
        if (ect.eEventId == SPEI_FALSE_RECOGNITION || *resultConfidence < confidenceThreshold) //get rid of false recognitions
        {
            *understoodPhrase = "Didn't understand Phrase";
            *matchedKeyword = notUnderstoodKeyword;
            resultKeywordList->clear();
        }
    }
    else
    {
        *understoodPhrase = "No speech";
        *matchedKeyword = noSpeechKeyword;
        resultKeywordList->clear();
        *resultConfidence = -1.0f;
    }
}

void SpeechRecognition::DecomposeSpeech(const SPPHRASEPROPERTY* data, std::vector<std::string>* keywords, std::vector<float>* confidences)
{
    //Add toplevel element keyword and confidence
    keywords->push_back(StringOperations::LPCW2stdstring(data->pszValue));
    confidences->push_back(data->SREngineConfidence);
    //decompose children and append result
    if (data->pFirstChild != NULL)
    {
        DecomposeSpeech(data->pFirstChild, keywords, confidences);
    }
    //decompose next siblings and append result
    if (data->pNextSibling != NULL)
    {
        DecomposeSpeech(data->pNextSibling, keywords, confidences);
    }
}


SpeechRecognition::~SpeechRecognition()
{
    cpGram.Release();
    cpRecoCtx.Release();
    cpEngine.Release();

    //remove( const char *path );

    CoUninitialize();
}