#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: NeuralNet

Description: A NeuralNet written in TensorFlow
'''
import numpy as np
from Regression import regression
import tensorflow as tf
from DataSet import DataSet


def identity(something):
    return something
    
layersDict={"sigmoid" : tf.nn.sigmoid, "tanh" : tf.nn.tanh, "softmax" : tf.nn.softmax, "relu" : tf.nn.relu, "linear" : identity}

class NeuralNetTF(regression):
    def __init__(self, rs):
        regression.__init__(self,rs)
        self.learningRate=rs.learningRate
        self.momentum=rs.momentum
        self.nbW=0
        self.nbB=0
        
        self.x = tf.placeholder(tf.float32, shape=[None, rs.inputDim])
        self.y_ = tf.placeholder(tf.float32, shape=[None, rs.outputDim])

        #no hidden Layer
        if(len(rs.hiddenLayers)==0):
            #connection between input and output Layer
            in_to_out = [self.weight_variable([rs.inputDim,rs.outputDim]),self.bias_variable([rs.outputDim])]
            self.y = layersDict[rs.outputLayer](tf.matmul(self.x,in_to_out[0]) + in_to_out[1])
            self.listTheta=[in_to_out]
            self.listBias=[in_to_out[1]]
            self.listWeight=[in_to_out[0]]
            self.theta = np.empty(rs.inputDim*rs.outputDim+rs.outputDim)
        else :
            #hidden Layers
            self.listTheta=[]
            self.listBias=[]
            self.listWeight=[]
            precDim=rs.inputDim
            y=self.x
            size=0
            for layer in rs.hiddenLayers:
                tmp=[self.weight_variable([precDim,layer[1]]),self.bias_variable([layer[1]])]
                self.listTheta.append(tmp)
                self.listBias.append(tmp[1])
                self.listWeight.append(tmp[0])
                y=layersDict[layer[0]](tf.matmul(y,tmp[0]) + tmp[1])
                size+=precDim*layer[1]+layer[1]
                precDim=layer[1]
             
            tmp=[self.weight_variable([precDim,rs.outputDim]),self.bias_variable([rs.outputDim])]
            self.listTheta.append(tmp)
            self.listBias.append(tmp[1])
            self.listWeight.append(tmp[0])
            self.y=layersDict[rs.outputLayer](tf.matmul(y,tmp[0]) + tmp[1])
            size+=precDim*rs.outputDim+rs.outputDim
            self.theta=np.empty(size)
            
            
        self.meanSquareError = tf.reduce_mean(tf.square(tf.sub(self.y_,self.y))) 
        self.train_step = tf.train.AdamOptimizer(rs.learningRate).minimize(self.meanSquareError)
        
        self.init_op = tf.initialize_all_variables()
        self.saver = tf.train.Saver(self.listBias+self.listWeight)
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
        load weights of the neural network from the thetafile
        '''
        self.saver.restore(self.sess,thetaFile+".ckpt")
        #print ("loaded theta : ", self.theta)

    def setTheta(self, theta):
        crt=0
        for W, b in self.listTheta:
            row = W.get_shape()[0].value
            column=W.get_shape()[1].value
            W=W.assign(np.reshape(theta[crt:crt+row*column],(row,column)))
            self.sess.run(W)
            crt+=row*column
            b=b.assign(theta[crt:crt+b.get_shape()[0].value])
            self.sess.run(b)
            crt+=b.get_shape()[0].value

    def getTheta(self):
        crt=0
        for W, b in self.listTheta :
            for row in W.eval(session=self.sess):
                self.theta[crt:crt+row.shape[0]]=row
                crt+=row.shape[0]
            bEval=b.eval(session=self.sess)
            self.theta[crt:crt+bEval.shape[0]]=bEval
            crt+=bEval.shape[0]
        return self.theta  
 
                
            
    def train(self):
        minError = 1000
        for i in range(100000):
            #batch = self.data.next_batch(1000)
            #self.train_step.run(feed_dict={self.x: batch[0], self.y_: batch[1]}, session=self.sess)
            self.train_step.run(feed_dict={self.x: self.data.inputData, self.y_: self.data.outputData}, session=self.sess)
            if(i%10==0):
                error = self.meanSquareError.eval(session=self.sess,feed_dict={self.x: self.data.inputData, self.y_: self.data.outputData})
                print(error) 
                if error < minError :
                    self.saver.save(self.sess, self.rs.path+self.rs.thetaFile+".ckpt") 
                    self.saveTheta(self.rs.path+self.rs.thetaFile+".theta")
                    minError = error

                
     
    def weight_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev=0.1)
        self.nbW+=1
        return tf.Variable(initial, name="Weight"+str(self.nbW))

    def bias_variable(self, shape):
        initial = tf.constant(0.1, shape=shape)
        self.nbB+=1
        return tf.Variable(initial,name="bias"+str(self.nbB))
           
    def computeOutput(self, inputData):
        result = self.sess.run(self.y, feed_dict={self.x: [inputData]})
        return result[0]
    
