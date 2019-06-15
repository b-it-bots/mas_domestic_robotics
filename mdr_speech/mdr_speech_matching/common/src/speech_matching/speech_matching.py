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

        self.gt_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../', 'config/'))

        self.available_files = []
        for filename in os.listdir(self.gt_dir):
            if filename.endswith(".txt"):
                self.available_files.append(filename)

    """
    The following method creates a sentence pool of a text file containing a
    list of words.
    """
    @staticmethod
    def load_pool(filename):
        file = open(filename, "r")
        sentences = []
        for line in file:
            sentences.append(line.strip())
        return sentences

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
        for filename in self.available_files:
            file_dir = os.path.join(self.gt_dir, filename)
            sentence_pool = SpeechMatching.load_pool(file_dir)
            sentences_match = process.extract(input_sentence, sentence_pool)
            if sentences_match[1] >= self.threshold:
                return True
        return False

    """
    The following method returns a matching sentence, its Levenshtein distance
    and the filename in which the match can be found. In case the
    match is below the threshold, the return sentence is empty, the distance is
    zero and the category is 'nothing'.
    """
    def match_sentence(self, input_sentence):
        best_match = []
        best_match_filename = ""
        for filename in self.available_files:
            file_dir = os.path.join(self.gt_dir, filename)
            sentence_pool = SpeechMatching.load_pool(file_dir)
            sentences_match = process.extractOne(input_sentence, sentence_pool)
            if sentences_match[1] >= self.threshold:
                if len(best_match) != 0:
                    if best_match[1] < sentences_match[1]:
                        best_match = sentences_match
                        best_match_filename = filename
                else:
                    best_match = sentences_match
                    best_match_filename = filename
        if len(best_match) != 0:
            return [best_match_filename, best_match]
        else:
            return ["nothing", ("", 0)]
