# mdr_sound_communication

A simple monitor node which plays a sound from the willow-sounds collection when requested.

There are two way to play a sound:

* Monitorable topics: Every time a node published to the topic, this node plays a defined sound.
* Sendind a Request: Any node can request to play a sound by publishing the sound_Id on topic '/sound_monitor'.


## Directory structure

```
mdr_sound_communication
├── CMakeLists.txt
├── config
│   ├── actions_config.yaml
│   ├── alarms_config.yaml
│   ├── services_config.yaml
│   ├── states_config.yaml
│   └── topic_config.yaml
├── package.xml
├── README.md
├── ros
│   ├── cfg
│   │   └── soundCommunication.cfg
│   ├── launch
│   │   └── sound_test.launch
│   ├── scripts
│   │   └── sound_communication
│   └── src
│       └── mdr_sound_communication
│           ├── __init__.py
│           ├── sound_communication.py
├── setup.py
└── willow-sound
   ├── A
   │   ├── ABeepProcessing.wav
   │   ├── ABeepQuizical.wav
   │   ├── AError2.wav
   │   ├── AFailure.wav
   │   ├── AFinished1.wav
   │   ├── AHelp.wav
   │   ├── AHey.wav
   │   ├── AHmm1.wav
   │   ├── AHmm2.wav
   │   ├── AHuh2.wav
   │   ├── AOK.wav
   │   ├── AOneMomentPlease.wav
   │   ├── AReady.wav
   │   └── ASorry.wav
   ├── B
   │   ├── BBeepSuccess.wav
   │   ├── BBeepThinking.wav
   │   ├── BGoAhead.wav
   │   ├── BHmm.wav
   │   ├── BPleaseHelp2.wav
   │   └── BThinking.wav
   ├── C
   │   ├── C2BeepsFast.wav
   │   ├── C2LowBeepFast.wav
   │   ├── C3BeepFast.wav
   │   ├── CAttention.wav
   │   ├── CBeep2Tone.wav
   │   ├── CBeepVary.wav
   │   ├── CBmmmm.wav
   │   ├── CDeBeep.wav
   │   ├── CFastPulse.wav
   │   ├── CKaching2.wav
   │   ├── CKaching3.wav
   │   ├── CKachingFailed.wav
   │   ├── CMrnful2.wav
   │   ├── CPlugMeIn1.wav
   │   ├── CPlugMeIn2.wav
   │   └── CPlugMeIn3.wav
   ├── D
   │   ├── D01a.wav
   │   ├── D02o.wav
   │   ├── D02p.wav
   │   ├── D04.wav
   │   ├── D05.wav
   │   ├── D09.wav
   │   ├── D12.wav
   │   ├── D17.wav
   │   ├── D20.wav
   │   ├── D23.wav
   │   ├── D32.wav
   │   ├── DOuch.wav
   │   └── DRecvCmmnd2.wav
   ├── E
   │   ├── E01.wav
   │   ├── E06.wav
   │   ├── E07.wav
   │   ├── E09.wav
   │   └── E12.wav
   ├── F
   │   ├── F01.wav
   │   ├── F06.wav
   │   ├── F09.wav
   │   ├── F10.wav
   │   ├── F12.wav
   │   └── F14.wav
   ├── G
   │   ├── G01.wav
   │   ├── G03.wav
   │   ├── G11.wav
   │   ├── G17.wav
   │   ├── G20.wav
   │   ├── G24.wav
   │   ├── G25.wav
   │   ├── G27b.wav
   │   ├── G28.wav
   │   ├── G37.wav
   │   └── G38.wav
   ├── H
   │   ├── H05.wav
   │   └── H07.wav
   ├── I
   │   ├── I01.wav
   │   ├── I0h2.wav
   │   ├── IAck1.wav
   │   ├── IAttention1.wav
   │   ├── IAttn1.wav
   │   ├── IBatWarn4.wav
   │   ├── IDiag2.wav
   │   ├── IDiag3.wav
   │   ├── IDiagAlrt1.wav
   │   ├── IErrgh1.wav
   │   ├── IGotIt.wav
   │   ├── IHey2.wav
   │   ├── IITried1.wav
   │   ├── IMmm2.wav
   │   ├── ISad1.wav
   │   ├── IWohoo1.wav
   │   └── IWooHoo1.wav
   ├── J
   │   ├── JErrgh.wav
   │   ├── JHey2.wav
   │   ├── JHey.wav
   │   ├── JHm.wav
   │   ├── JOh.wav
   │   ├── JOops2.wav
   │   ├── JOops.wav
   │   ├── JWooHoo2.wav
   │   └── JWooHoo.wav
   ├── K
   │   ├── KAw.wav
   │   ├── KDuh.wav
   │   ├── KHey-A1.wav
   │   ├── KOh.wav
   │   ├── KUgHa.wav
   │   └── KUgOh-C#3.wav
   ├── L
   │   ├── L02.wav
   │   ├── L05-D#4.wav
   │   └── L24-E5.wav
   ├── LICENSE
   ├── M
   │   ├── MCalculating.wav
   │   ├── MConfirmed.wav
   │   ├── MErEr.wav
   │   ├── MError.wav
   │   ├── MGotIt.wav
   │   ├── MIDidIt.wav
   │   ├── MIDontUnderstand.wav
   │   ├── MOneMomentPlease.wav
   │   ├── MOneMoment.wav
   │   ├── MProcessing.wav
   │   ├── MSorry.wav
   │   ├── MStandby.wav
   │   ├── MSuccess.wav
   │   ├── MTaskCompleted.wav
   │   ├── MThinking.wav
   │   ├── MWaiting.wav
   │   ├── MWeHaveAProblem.wav
   │   └── MYouGotIt.wav
   └── README.md

```

## Launch file parameters

* ``config_files``: A string with the names of the configuration files which will be loaded to the sound_communication

## Config files examples:

* Monitorable Topics:

<sound_id>:
  topic: <topic_name>
  file: <path_to_sound>

* Loading Sound Id without a topic:

<sound_id>: <path_to_sound>
