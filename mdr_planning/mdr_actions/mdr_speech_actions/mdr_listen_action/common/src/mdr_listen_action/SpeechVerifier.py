from difflib import SequenceMatcher

class SpeechVerifier:
    """ Analyzes the output of Julius speech recognition by comparing the sentences and their phonemes with the ones
    from a pool.
    
    This class takes the recognized phrases of Julius speech recognition and appends an additional analysis
    to improve the results. Prerequisite for the analysis is a pool of sentences. These sentences are assumed
    to be the only valid sentences (please see robocup rule book for further details).
    
    Attributes:
        path_dictionary: Shows where the text file lies, which contains the words and phonemes known by Julius.
        path_sentence_pool: Represents the path to the text file with the valid sentences.
        input_sentences: These are the output sentences of the Julius recognition.
        
        __phoneme_dictionary: A dictionary that represents the list of words and corresponding phonemes of all
            words known by Julius.
        __sentence_pool: A list containing the sentences from the pool of possible sentences.
        __sentence_pool_as_phonemes: A list that contains the sentences from the pool expressed in phonemes.
        __input_sentences: A list containing the sentences of the Julius recognition.
        __input_sentences_as_phonemes: A list that contains the input sentences as phonemes.
        __best_match_information_phonemes = A list of tuples that show how much the phonemes of the input sentences
            match the sentences of the pool. The first element of the tuple is a number representing an index
            that refers to the sentence of the pool with the highest similarity. The second value is the 
            similarity as a float in the range [0, 1].
        __best_match_information_sentences = A list of tuples that show how much the input sentences
            match the sentences of the pool. The first element of the tuple is a number representing an index
            that refers to the sentence of the pool with the highest similarity. The second value is the 
            similarity as a float in the range [0, 1].
    """
    
    def __init__(self, path_dictionary, path_sentence_pool, input_sentences):
        self.__phoneme_dictionary = self.generate_phoneme_dictionary(path_dictionary)
        self.__sentence_pool = open(path_sentence_pool, 'r').readlines()
        self.__sentence_pool_as_phonemes = self.transform_words_to_phonemes(self.__sentence_pool)
        self.__input_sentences = input_sentences
        self.__input_sentences_as_phonemes = self.transform_words_to_phonemes(input_sentences)

        self.__best_match_information_phonemes = self.compare()
        self.__best_match_information_sentences = self.compare(False)
    
    def generate_phoneme_dictionary(self, path_dictionary):
        """Generates a dictionary from the Julius 'dict' file.
        
        The 'dict' file is a necessary file for Julius speech recognition. It contains a list of words and
        their phonemes, that have been taught the system. This file is taken and a dictionary of the structure
        {word:phoneme} is generated.
        
        Args:
            path_dictionary: The path to the location of the Julius 'dict' file.
        
        Returns:
            A dictionary mapping words and phonemes. Both are represented as strings.
        """
        file_dictionary = open(path_dictionary, 'r')
        lines = file_dictionary.readlines()
        phoneme_dictionary = {}
        for line in lines:
            word = line.split(' ')[0].lower()
            phonemes = line.split(']')[1].lstrip().rstrip()
            phoneme_dictionary[word] = phonemes
        file_dictionary.close()
        return phoneme_dictionary
    
    def transform_words_to_phonemes(self, sentences):
        """Translates the words of a sentence in the corresponding phonemes.
        
        The sentences passed to this function are translated into their phoneme representation. Only words,
        from the Julius 'dict' file can be used in the sentences. The reason for that is, that the 'dict' file
        is used as reference work for the phonemes.
        
        Args:
            sentences: A list of sentences given as strings.
            
        Returns:
            A list of strings that represent the input sentences as phonemes.        
        """
        sentences_expressed_as_phonemes = []
        splitted_sentences = [sentence.split() for sentence in sentences]
        
        for splitted_sentence in splitted_sentences:
            phoneme_sequence = ""
            for word in splitted_sentence:
                phoneme_sequence += self.__phoneme_dictionary[word.lower()] + " "
                
            sentences_expressed_as_phonemes.append(phoneme_sequence)

        return sentences_expressed_as_phonemes
    
    def compare(self, phonemes=True):
        """Compares the similarity between input sentences and the sentences from the pool.
        
        This function enables the comparison of both, sentences in plain words and in phonemes.
        It takes the input sentences of the class and compares them to the sentences of the pool.
        The string comparison is solved with the help of the library 'difflib' and the class 'SequenceMatcher'.
        
        Args:
            phonemes(default True): This parameter decides whether the input sentences are compared in plain
                words or in phonemes. By default they are compared in phoneme form.
        Returns:
            A list of tuples. The first element of the tuple is an integer number representing a sentence
            from the pool. This reference indicates which sentence has the highest similarity in comparison with
            the input sentence at the same position of the list. The second element of the tuple is the similarity
            as a float in the range [0, 1].        
        """
        sm = SequenceMatcher()
        best_match_information = []

        if phonemes:
            for input_sentence in self.__input_sentences_as_phonemes:
                sm.set_seq1(input_sentence)
                match = best_match = 0
                best_match_sentence = None

                index = 0
                for sentence in self.__sentence_pool_as_phonemes:
                    sm.set_seq2(sentence)
                    match = sm.ratio()
                    if (match >= best_match):
                        best_match = match
                        best_match_sentence = index
                    index += 1
                best_match_information.append((best_match_sentence, best_match))
        else:
            for input_sentence in self.__input_sentences:
                sm.set_seq1(input_sentence)
                match = best_match = 0
                best_match_sentence = None
                
                index = 0
                for sentence in self.__sentence_pool:
                    sm.set_seq2(sentence)
                    match = sm.ratio()
                    if (match >= best_match):
                        best_match = match
                        best_match_sentence = index
                    index += 1
                best_match_information.append((best_match_sentence, best_match))
                
        return best_match_information
    
    def find_best_match(self):
        """Takes the similarity information and finds the sentence with the most promising results.
        
        The function uses the similarity information from phonemes and plain words, computed by the help of
        the 'compare' function, and finds the most similar sentence from the pool. If the output from
        the phoneme and sentence analysis differ then the similarity is set 0. The same applies if the similarity
        from the sentence analysis is smaller than the set treshold. Otherwise the similarities of the
        phonemes and sentences analysis are summed up. Winner is the sentence with the greates similarity.
        
        Returns:
            The sentence with the greatest similarity is returned. The base for this conclusion are the
            recognized sentences of Julius speech recognition.        
        """
        sentences, sentences_similarities = zip(*self.__best_match_information_sentences)
        sentences = list(sentences)
        sentences_similarities = list(sentences_similarities)
        
        phonemes, phonemes_similarities = zip(*self.__best_match_information_phonemes)
        phonemes_similarities = list(phonemes_similarities)
        
        summed_similarities = []
        for x in range(len(self.__input_sentences)):
            if sentences[x] != phonemes[x]:
                summed_similarities.append(0)
            elif sentences_similarities[x] < 0.5:
                summed_similarities.append(0)
            else:
                summed_similarities.append(sentences_similarities[x] + phonemes_similarities[x])
        
        similarity_best_match = max(summed_similarities)
        if similarity_best_match == 0:
            return "Sentence not recognized!"
        else:
            index_of_best_match = summed_similarities.index(similarity_best_match)
            recognized_sentence = self.__sentence_pool[sentences[index_of_best_match]]
            return recognized_sentence.rstrip()

