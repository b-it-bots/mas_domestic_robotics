import os
from abc import ABCMeta, abstractmethod
from utils import get_classes_in_data_dir


class ImageClassifier(object):
    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self._classes = kwargs.get('classes', None)

        if self._classes is None:
            class_file = kwargs.get('class_file', None)
            if class_file is not None and os.path.exists(class_file):
                self._classes = ImageClassifier.read_classes_from_file(class_file)

        if self._classes is None:
            data_dir = kwargs.get('data_dir', None)
            if data_dir is None:
                raise ValueError('no class definition specified')
            if not os.path.exists(data_dir):
                raise ValueError('Directory does not exist: ' + data_dir)

            self._classes = get_classes_in_data_dir(data_dir)
        pass

    @property
    def classes(self):
        return self._classes

    @abstractmethod
    def classify(self, image_messages):
        pass

    @staticmethod
    def write_classes_to_file(classes, outfile_path):
        with open(outfile_path, 'w') as outfile:
            outfile.write('\n'.join(classes))

    @staticmethod
    def read_classes_from_file(infile):
        with open(infile) as f:
            content = f.readlines()
            return [x.strip() for x in content]


class ImageClassifierTest(ImageClassifier):
    def __init__(self, **kwargs):
        super(ImageClassifierTest, self).__init__(**kwargs)

    def classify(self, image_messages):
        import random
        indices = list(range(len(image_messages)))
        classes = [self.classes[random.randint(0, len(self.classes) - 1)] for _ in indices]
        probabilities = [random.random() for _ in indices]
        return indices, classes, probabilities
