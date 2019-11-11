from tensorflow.keras.models import model_from_json
from tensorflow.keras.utils import CustomObjectScope
from tensorflow.keras.initializers import glorot_uniform
from tensorflow import keras
import tensorflow as tf
from utils.param import Param
import numpy as np


class Model:
    def __init__(self, path=None, weights=None):
        self.session = tf.Session()
        keras.backend.set_session(self.session)
        if path is None:
            path = Param().model_lane_path
        if weights is None:
            weights = Param().model_lane_weights
        f = open(path, 'r')
        with CustomObjectScope({'GlorotUniform': glorot_uniform()}):
            self.model = model_from_json(f.read())
        self.model.load_weights(weights)
        self.model._make_predict_function()

    def predict(self, img):
        with self.session.as_default():
            with self.session.graph.as_default():
                img = np.expand_dims(img, axis=0)
                pred = self.model.predict(img/255.)
                pred = np.argmax(pred, axis=3)[0]
                return pred

