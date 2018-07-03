from abc import ABCMeta, abstractmethod
from enum import Enum


class RgbDetectionKey(Enum):
    CLASS = 'class'
    CONF = 'confidence'
    X_MIN = 'x_min'
    X_MAX = 'x_max'
    Y_MIN = 'y_min'
    Y_MAX = 'y_max'


class RgbDetector(object):
    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self.load_model(**kwargs)

    @abstractmethod
    def load_model(self, **kwargs):
        pass

    @abstractmethod
    def detect(self, image):
        pass
