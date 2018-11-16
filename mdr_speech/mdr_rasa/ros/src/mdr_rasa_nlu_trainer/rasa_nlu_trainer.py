#!/usr/bin/env python

import rospy
import os
from rasa_nlu.training_data import load_data
from rasa_nlu.model import Trainer
from rasa_nlu import config


class RasaNluTrainer(object):
    def __init__(self):
        # Setup rospy and get config
        rospy.init_node('rasa_nlu_wrapper')
        self.model_name = rospy.get_param('~rasa_nlu_model', 'restaurant_sample')

        # Build paths to rasa models
        self.base_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../../models', self.model_name))
        self.training_filepath = os.path.join(self.base_path, 'training_data.md')
        self.config_filepath = os.path.join(self.base_path, 'config.yml')

    def run(self):
        rospy.loginfo('Training model {}...'.format(self.model_name))

        training_data = load_data(self.training_filepath)
        trainer = Trainer(config.load(self.config_filepath))
        trainer.train(training_data)
        model_directory = trainer.persist(self.base_path, project_name='generated', fixed_model_name='model')

        rospy.loginfo('Training done! Model directory: {}'.format(model_directory))
