#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: NeuralNet

Description: A NeuralNet to be trained as the arm controller
'''
import numpy as np
from Regression import regression
import tensorflow as tf
from DataSet import DataSet

layersDict={"sigmoid" : tf.nn.sigmoid, "tanh" : tf.nn.tanh, "softmax" : tf.nn.softmax, "relu" : tf.nn.relu}

class NeuralNetTF(regression):
    def __init__(self, rs):
        regression.__init__(self,rs)
        self.learningRate=rs.learningRate
        self.momentum=rs.momentum
        
        self.x = tf.placeholder(tf.float32, shape=[None, rs.inputDim])
        self.y_ = tf.placeholder(tf.float32, shape=[None, rs.outputDim])

        #no hidden Layer
        if(len(rs.hiddenLayers)==0):
            #connection between input and output Layer
            in_to_out = [self.weight_variable([rs.inputDim,rs.outputDim]),self.bias_variable([rs.outputDim])]
            self.y = layersDict[rs.outputLayer](tf.matmul(self.x,in_to_out[0]) + in_to_out[1])
            self.listTheta=[in_to_out]
            self.theta = np.empty(rs.inputDim*rs.outputDim+rs.outputDim)
        else :
            #hidden Layers
            self.listTheta=[]
            precDim=rs.inputDim
            self.y=self.x
            size=0
            for layer in rs.hiddenLayers:
                tmp=[self.weight_variable([precDim,layer[1]]),self.bias_variable([layer[1]])]
                self.listTheta.append(tmp)
                self.y=layersDict[layer[0]](tf.matmul(self.y,tmp[0]) + tmp[1])
                precDim=layer[1]
                size+=precDim*layer[1]+layer[1]
             
            tmp=[self.weight_variable([precDim,rs.outputDim]),self.bias_variable([rs.outputDim])]
            self.listTheta.append(tmp)
            self.y=layersDict[rs.outputLayer](tf.matmul(self.y,tmp[0]) + tmp[1])
            size+=precDim*rs.outputDim+rs.outputDim
            self.theta=np.empty(size)
            
            
        self.meanSquareError = tf.reduce_mean(tf.square(tf.sub(self.y_,self.y))) 
        self.train_step = tf.train.AdamOptimizer(rs.learningRate).minimize(self.meanSquareError)
        
        self.init_op = tf.initialize_all_variables()
        self.saver = tf.train.Saver()
        self.sess = tf.Session()
        self.sess.run(self.init_op)
        self.sess.as_default()
     
    def getTrainingData(self, inputData, outputData):
        '''
        Verifies the validity of the given input and output data
        Data should be organized by columns
        
        Input:      -inputdata, numpy N-D array
                    -outputData, numpy N-D array
        '''
        regression.getTrainingData(self,inputData, outputData) 
        self.data = DataSet(inputData, outputData)
        
    def load(self,thetaFile):
        '''
        load wheight of the neural network from the thetafile
        '''
        self.saver.restore(self.sess,thetaFile+".ckpt")
        #print ("theta LOAD : ", self.net.params)
        return self.net.params
    
    #TODO: sea that
    def setTheta(self, theta):
        crt=0
        for W, b in self.listTheta:
            self.W.asign(np.reshape(theta[crt:W.get_shape()[0]*W.get_shape()[1]],(W.get_shape()[0],W.get_shape()[1])))
            crt+=W.get_shape()[0]*W.get_shape()[1]
            self.b.asign(theta[crt:b.get_shape()[0]])
            crt+=b.get_shape()[0]
     
    def getTheta(self):
        crt=0
        for W, b in self.listTheta :
            for ligne in W:
                self.theta[crt:ligne.get_shape()[0]]=ligne
                crt+=ligne.get_shape()[0]
            self.theta[crt:b.get_shape()[0]]=b
            crt+=b.get_shape()[0]
        return self.theta      
            
    def train(self):
        
        for i in range(10000):
            #batch = self.data.next_batch(1000)
            #self.train_step.run(feed_dict={self.x: batch[0], self.y_: batch[1]}, session=self.sess)
            self.train_step.run(feed_dict={self.x: self.data.inputData, self.y_: self.data.outputData}, session=self.sess)
            if(i%10==0):
                print(self.meanSquareError.eval(session=self.sess,feed_dict={self.x: self.data.inputData, self.y_: self.data.outputData})) 
                self.saver.save(self.sess, self.rs.path+self.rs.thetaFile+".ckpt") 
                self.saveTheta(self.rs.path+self.rs.thetaFile+".theta")
                
     
    def weight_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev=0.1)
        return tf.Variable(initial)

    def bias_variable(self, shape):
        initial = tf.constant(0.1, shape=shape)
        return tf.Variable(initial)
           
    def computeOutput(self, inputData):
        if(len(inputData.shape)==1):
            inputData=inputData.reshape((1,inputData.shape[0]))
            one=True
        result = self.y.eval(session=self.sess, feed_dict={self.x: inputData})
        if one :
            return result[0]
        return result 