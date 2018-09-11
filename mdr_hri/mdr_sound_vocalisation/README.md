# mdr_sound_vocalisation

A package for functionalities related to speech-based robot communication.

## Existing Functionalities

### sound_vocaliser_base

A base implementation of a sound vocalisation functionality that can be used for speech synthesis and sound generation.

The base class defines the following methods for reacting to sound requests:
* `say`: Receives a sentence request and processes it so that speech is generated
* `make_sound`: Receives a sound request and processes it so that the requested sound is produced

Robot-specific implementations need to override both methods.

A script that starts a `sound_vocaliser` node is also included in the package as an example, but a robot-specific implementation should start its own node.

## Launch file parameters

### sound_vocalisation

The following parameters may be passed when launching the sound vocalisation node:
* ``speech_request_topic``: Topic on which speech requests should be sent (default '/say')
* ``sound_request_topic``: Topic on which sound requests should be setn (default '/make_sound')
* ``speech_topic``: Robot-specific speech synthesis topic (default '/sound/say')
* ``sound_topic``: Robot-specific topic to which sound requests should be forwarded so that sound is produced (default '/sound/make')

## Directory structure

```
mdr_sound_vocalisation
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____launch
     |    |_____sound_vocalisation.launch
     |    |
     |    scripts
     |    |_____sound_vocaliser
     |    |
     |____src
          |____mdr_sound_vocalisation
               |    __init__.py
               |____sound_vocalisation_base.py
```