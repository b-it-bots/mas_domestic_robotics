import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from utils import process_image_message
from image_classifier import ImageClassifier


class KerasImageClassifier(ImageClassifier):
    def __init__(self, **kwargs):
        import keras.backend as T
        from keras.models import Model, load_model

        T.clear_session()

        super(KerasImageClassifier, self).__init__(**kwargs)

        self._model = kwargs.get('model', None)
        model_path = kwargs.get('model_path', None)
        if self._model is None:
            if model_path is not None:
                self._model = load_model(model_path)
            else:
                raise ValueError('No model object or path passed received')

        if not isinstance(self._model, Model):
            raise ValueError('model is not a Keras Model object')

        if len(self._classes) != self._model.output_shape[-1]:
            raise ValueError('number of classes ({0}) does not match model output shape ({1})'
                             .format(len(self._classes), self._model.output_shape[-1]))

        self._img_preprocess_func = kwargs.get('img_preprocess_func', None)

        # assume input shape is 3D with channel dimension to be 3
        self._target_size = tuple(i for i in self._model.input_shape if i != 3 and i is not None)
        # CvBridge for ROS image conversion
        self._cv_bridge = CvBridge()

    def classify(self, image_messages):
        if len(image_messages) == 0:
            return [], [], []

        np_images = [process_image_message(msg, self._cv_bridge, self._target_size, self._img_preprocess_func)
                     for msg in image_messages]

        image_array = []
        indices = []
        for i in range(len(np_images)):
            if np_images[i] is None:
                continue

            image_array.append(np_images[i])
            indices.append(i)

        image_array = np.array(image_array)
        preds = self._model.predict(image_array)
        class_indices = np.argmax(preds, axis=1)
        confidences = np.max(preds, axis=1)
        predicted_classes = [self._classes[i] for i in class_indices]

        return indices, predicted_classes, confidences
