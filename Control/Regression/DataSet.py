#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: DataSet

Description: A DataSet class for NeuralNetTF
'''

import random
import numpy as np
from Regression import regression
import tensorflow as tf



class DataSet(object):

    def __init__(self, inputData, outputData):
        """
        Construct a DataSet. 
        
        Input :            -inputData, numpy array
                           -outputData, numpy array
        
        """

        assert inputData.shape[0] == outputData.shape[0], (
                'images.shape: %s labels.shape: %s' % (inputData.shape,
                                                       outputData.shape))
        self._num_examples = inputData.shape[0]

       
        self._input = inputData
        self._output = outputData
        self._epochs_completed = 0
        self._index_in_epoch = 0
        
    @property
    def inputData(self):
        return self._input

    @property
    def outputData(self):
        return self._output

    @property
    def num_examples(self):
        return self._num_examples

    @property
    def epochs_completed(self):
        return self._epochs_completed
    
    def next_batch(self, batch_size):
        """
        Return the next `batch_size` examples from this data set.
        """
    
        start = self._index_in_epoch
        self._index_in_epoch += batch_size
        if self._index_in_epoch > self._num_examples:
            # Finished epoch
            self._epochs_completed += 1
            # Shuffle the data
            perm = np.arange(self._num_examples)
            np.random.shuffle(perm)
            self._input = self._input[perm]
            self._output = self._output[perm]
            # Start next epoch
            start = 0
            self._index_in_epoch = batch_size
            assert batch_size <= self._num_examples
        end = self._index_in_epoch
        return self._input[start:end], self._output[start:end]