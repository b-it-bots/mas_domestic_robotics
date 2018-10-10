#!/usr/bin/env python
import os
from fuzzywuzzy import fuzz, process

"""
In order to match the input coming from speech recognition with a sentence in
our database, we are going to use a mix of the following methods:
- Levenshtein distance (fuzzywuzzy)
- Soundex (not implemented yet - probably fuzzy library)
- Double Metaphone (not implemented yet - probably fuzzy library)
"""

class SpeechMatching():

    def __init__(self, threshold=90):
        self.threshold = threshold

        question_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', 'config/questions.txt'))
        command_dir = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', 'config/commands.txt'))

        self.question_pool = self.load_pool(question_dir)
        self.question_sentences = [i[0].strip() for i in self.question_pool]
        self.question_phonemes = [i[1].strip() for i in self.question_pool]

        self.command_pool = self.load_pool(command_dir)
        self.command_sentences = [i[0].strip() for i in self.command_pool]
        self.command_phonemes = [i[1].strip() for i in self.command_pool]

    """
    The following method loads a sentence pool and returns a list of tuples,
    containing the readable sentence and the sentence in phonemes.
    """
    def load_pool(self, filename):
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
        question_match = process.extract(input_sentence, self.question_sentences)
        if question_match[1] >= self.threshold:
            return True
        command_match = process.extract(input_sentence, self.command_sentences)
        if command_match[1] >= self.threshold:
            return True
        return False

    """
    The following method returns a matching sentence, its Levenshtein distance
    and the category to which it belongs (question or command). In case the
    match is below the threshold, the return sentence is empty, the distance is
    zero and the category is 'nothing'.
    """
    def match_sentence(self, input_sentence):
        question_match = process.extractOne(input_sentence, self.question_sentences)
        command_match = process.extractOne(input_sentence, self.command_sentences)

        if question_match[1] >= self.threshold and question_match[1] >= command_match[1]:
            return ["question", question_match]
        elif command_match[1] >= self.threshold:
            return ["command", command_match]
        else:
            return ["nothing", ("", 0)]
