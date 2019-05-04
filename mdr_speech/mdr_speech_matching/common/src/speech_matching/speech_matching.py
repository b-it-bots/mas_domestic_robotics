#!/usr/bin/env python
import os
from fuzzywuzzy import process

"""
In order to match the input coming from speech recognition with a sentence in
our database, we are going to use a mix of the following methods:
- Levenshtein distance (fuzzywuzzy)
- Soundex (not implemented yet - probably fuzzy library)
- Double Metaphone (not implemented yet - probably fuzzy library)
"""

class SpeechMatching(object):

    def __init__(self, threshold=90):
        self.threshold = threshold

        objects_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', 'config/objects.txt'))
        locations_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', 'config/locations.txt'))

        self.objects_pool = SpeechMatching.load_pool(objects_dir)
        self.objects_sentences = [i[0].strip() for i in self.objects_pool]
        self.objects_phonemes = [i[1].strip() for i in self.objects_pool]

        self.locations_pool = SpeechMatching.load_pool(locations_dir)
        self.locations_sentences = [i[0].strip() for i in self.locations_pool]
        self.locations_phonemes = [i[1].strip() for i in self.locations_pool]

    """
    The following method loads a sentence pool and returns a list of tuples,
    containing the readable sentence and the sentence in phonemes. This method
    is similar to the method used in objects_responder (TODO: export this
    method and use it in both.)
    """
    @staticmethod
    def load_pool(filename):
        file = open(filename, "r")
        sentences_and_phonemes = []
        for line in file:
            sentence_and_phoneme = line.split(":")
            sentences_and_phonemes.append(sentence_and_phoneme)
        return sentences_and_phonemes

    """
    The following method changes the initially set threshold.
    """
    def set_threshold(self, new_threhold_value):
        self.threshold = new_threhold_value

    """
    The following method returns a boolean value.
    If there is at least one sentence in one of the two sentence pools with a
    Levenshtein distance equal or greater than the threshold then this method
    returns True otherwise False.
    """
    def find_match(self, input_sentence):
        objects_match = process.extract(input_sentence, self.objects_sentences)
        if objects_match[1] >= self.threshold:
            return True
        locations_match = process.extract(input_sentence, self.locations_sentences)
        if locations_match[1] >= self.threshold:
            return True
        return False

    """
    The following method returns a matching sentence, its Levenshtein distance
    and the category to which it belongs (objects or locations). In case the
    match is below the threshold, the return sentence is empty, the distance is
    zero and the category is 'nothing'.
    """
    def match_sentence(self, input_sentence):
        objects_match = process.extractOne(input_sentence, self.objects_sentences)
        locations_match = process.extractOne(input_sentence, self.locations_sentences)

        if objects_match[1] >= self.threshold and objects_match[1] >= locations_match[1]:
            return ["objects", objects_match]
        elif locations_match[1] >= self.threshold:
            return ["locations", locations_match]
        else:
            return ["nothing", ("", 0)]
