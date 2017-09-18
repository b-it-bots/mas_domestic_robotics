__author__ = 'Iryna'
#original code is taken from http://stackoverflow.com/questions/892199/detect-record-audio-in-python

'''NOTE: All words in grammar file should be in sentences.py and vise versa'''
'''RoboCup2016: There are 50 questions given. Give an answer to 5 out of 50 in 5 min.'''

import os
from array import array
from struct import pack
from sys import byteorder

import copy
import pyaudio
import wave


THRESHOLD = 5000 #5000  # audio levels not normalised.
CHUNK_SIZE = 1024
SILENT_CHUNKS = 3 * 44100 / 1024  # about 10sec?
FORMAT = pyaudio.paInt16
FRAME_MAX_VALUE = 2 ** 15 - 1
NORMALIZE_MINUS_ONE_dB = 10 ** (-1.0 / 20)
RATE = 44100
CHANNELS = 1 # NOTE: 2 channels will not work with usb mic
TRIM_APPEND = RATE / 4


def is_silent(data_chunk):
    # returns 'True' if below the 'silent' threshold
    if max(data_chunk) > THRESHOLD:
        print max(data_chunk), max(data_chunk) < THRESHOLD
    return max(data_chunk) < THRESHOLD


def normalize(data_all):
    # amplify the volume out to max -1dB
    # MAXIMUM = 16384
    normalize_factor = (float(NORMALIZE_MINUS_ONE_dB * FRAME_MAX_VALUE)
                        / max(abs(i) for i in data_all))

    r = array('h')
    for i in data_all:
        r.append(int(i * normalize_factor))
    return r


def trim(data_all):
    _from = 0
    _to = len(data_all) - 1
    for i, b in enumerate(data_all):
        if abs(b) > THRESHOLD:
            _from = max(0, i - TRIM_APPEND)
            break

    for i, b in enumerate(reversed(data_all)):
        if abs(b) > THRESHOLD:
            _to = min(len(data_all) - 1, len(data_all) - 1 - i + TRIM_APPEND)
            break

    return copy.deepcopy(data_all[_from:(_to + 1)])


def record():
    # record a word or words from the microphone and return the data as an array of signed shorts.
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, output=True, frames_per_buffer=CHUNK_SIZE)

    silent_chunks = 0
    audio_started = False
    data_all = array('h')

    while True:
        # little endian, signed short
        data_chunk = array('h', stream.read(CHUNK_SIZE))
        if byteorder == 'big':
            data_chunk.byteswap()
        data_all.extend(data_chunk)

        silent = is_silent(data_chunk)

        if audio_started:
            if silent:
                silent_chunks += 1
                if silent_chunks > SILENT_CHUNKS:
                    break
            else:
                silent_chunks = 0
        elif not silent:
            audio_started = True

    sample_width = p.get_sample_size(FORMAT)
    stream.stop_stream()
    stream.close()
    p.terminate()

    # we trim before normalize as thresh hold applies to un-normalized wave (as well as is_silent() function)
    data_all = trim(data_all)
    data_all = normalize(data_all)
    return sample_width, data_all


def record_to_file(path):
    # records from the microphone and outputs the resulting data to 'path'
    sample_width, data = record()
    data = pack('<' + ('h' * len(data)), *data)


    wave_file = wave.open(path, 'wb')
    print wave_file
    wave_file.setnchannels(CHANNELS)

    wave_file.setsampwidth(sample_width)
    wave_file.setframerate(RATE)
    wave_file.writeframes(data)
    wave_file.close()


"""
# test
#usb_location should be updated

usb_location = '/home/iryna/Desktop'

i = 1
while i < 6:
    file_name = "speech_audio_" + str(i) + ".wav"
    file_path = os.path.join(usb_location, file_name)
    record_to_file(file_path)
    i = i + 1

"""