#!/bin/sh
## Rode
arecord --device=hw:4,0 --format S16_LE --rate 48000 -c1 test.wav
## built in mic
#arecord --device=hw:0,0 --rate 16000  -c 4 test.wav
