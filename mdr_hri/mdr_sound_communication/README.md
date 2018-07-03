# mdr_sound_communication

A simple monitor node which plays a sound from the willow-sounds collection when requested.

There are two way to play a sound:

* Monitorable topics: Every time a node published to the topic, this node plays a defined sound.
* Sendind a Request: Any node can request to play a sound by publishing the sound_Id on topic '/sound_monitor'.


## Directory structure

```
mdr_sound_communication/
├── CMakeLists.txt
├── config
│   ├── actions_config.yaml
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
    ├── BBeepSuccess.wav
    ├── BBeepThinking.wav
    ├── BGoAhead.wav
    ├── BHmm.wav
    ├── BPleaseHelp2.wav
    ├── E01.wav
    ├── E06.wav
    ├── E07.wav
    ├── E09.wav
    ├── E12.wav
    ├── LICENSE
    └── README.md
```

## Launch file parameters

* ``config_files``: A string with the names of the configuration files which will be loaded to the sound_communication
* ``sound_collection``: Specify a sound collection, default is willow-sound. All sounds must be contained inside the sound_collection folder. If additional sound collections are added, the sound collection folders must be located on the mdr_sound_communication folder.

## Config files examples:

* Monitorable Topics:

<sound_id>:
  topic: <topic_name>
  file: <path_to_sound_in_sound_collection_folder>

* Loading Sound Id without a topic:

<sound_id>: <path_to_sound_in_sound_collection_folder>
