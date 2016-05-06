# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 14:51:47 2016

@author: debroissia
"""
from actor_network import actor_network
import math

import tensorflow as tf

class simple_actor_network(actor_network):
    l1_size = 20
    l2_size = 10
    learning_rate = 0.001
    ts = 0.001
    """A first actor network for low-dim state"""
    def __init__(self, state_size, action_size,l1_size = 20, l2_size = 10):
        
        self.graph = tf.Graph()
        with self.graph.as_default():
            self.sess = tf.Session()
        
            self.state_input = tf.placeholder(tf.float32, [None, state_size])
    
            self.W1 = tf.Variable(tf.random_uniform([state_size, l1_size], -1/math.sqrt(state_size), 1/math.sqrt(state_size)))
            self.W2 = tf.Variable(tf.random_uniform([l1_size, l2_size], -1/math.sqrt(l1_size), 1/math.sqrt(l1_size)))
            self.W3 = tf.Variable(tf.random_uniform([l2_size, action_size], -0.0003, 0.0003))
    
            self.b1 = tf.Variable(tf.random_uniform([l1_size], -1/math.sqrt(state_size), 1/math.sqrt(state_size)))
            self.b2 = tf.Variable(tf.random_uniform([l2_size], -1/math.sqrt(l1_size), 1/math.sqrt(l1_size)))
            self.b3 = tf.Variable(tf.random_uniform([action_size], -0.0003, 0.0003))
    
            self.W1_target = tf.Variable(tf.zeros([state_size, l1_size]))
            self.W2_target = tf.Variable(tf.zeros([l1_size, l2_size]))
            self.W3_target = tf.Variable(tf.zeros([l2_size, action_size]))
            
            self.b1_target = tf.Variable(tf.zeros([l1_size]))
            self.b2_target = tf.Variable(tf.zeros([l2_size]))
            self.b3_target = tf.Variable(tf.zeros([action_size]))
            
            
            self.x1 = tf.nn.softplus(tf.matmul(self.state_input,self.W1) + self.b1)
            self.x2 = tf.nn.tanh(tf.matmul(self.x1,self.W2) + self.b2 + tf.random_normal([l2_size], stddev=0.25))
            self.action_output = tf.matmul(self.x2,self.W3) + self.b3
            
            
    
            self.x1_target = tf.nn.softplus(tf.matmul(self.state_input,self.W1_target) + self.b1_target)
            self.x2_target = tf.nn.tanh(tf.matmul(self.x1_target,self.W2_target) + self.b2_target)
            self.action_output_target = tf.matmul(self.x2_target,self.W3_target) + self.b3_target
            
            self.action_gradient = tf.placeholder(tf.float32, [None, action_size])
            self.params = [self.W1, self.W2, self.W3, self.b1, self.b2, self.b3]
            self.params_grad = tf.gradients(self.action_output, self.params, -self.action_gradient) 
            
            self.adam = tf.train.AdamOptimizer(simple_actor_network.learning_rate)        
            self.optimizer = tf.train.GradientDescentOptimizer(simple_actor_network.learning_rate)
            self.updater = self.adam.apply_gradients(zip(self.params_grad, self.params))        
            
            init = tf.initialize_all_variables()                
            self.sess.run(init)
            
            self.sess.run([self.W1_target.assign(self.W1),
                           self.W2_target.assign(self.W2),
                           self.W3_target.assign(self.W3),
                           self.b1_target.assign(self.b1),
                           self.b2_target.assign(self.b2),
                           self.b3_target.assign(self.b3) ])
            
            
            self.upTargW1 = self.W1_target.assign(self.W1_target*(1-simple_actor_network.ts)+ self.W1*(simple_actor_network.ts))
            self.upTargW2 = self.W2_target.assign(self.W2_target*(1-simple_actor_network.ts)+ self.W2*(simple_actor_network.ts))        
            self.upTargW3 = self.W3_target.assign(self.W3_target*(1-simple_actor_network.ts)+ self.W3*(simple_actor_network.ts))
            
            self.upTargb1 = self.b1_target.assign(self.b1_target*(1-simple_actor_network.ts)+ self.b1*(simple_actor_network.ts))
            self.upTargb2 = self.b2_target.assign(self.b2_target*(1-simple_actor_network.ts)+ self.b2*(simple_actor_network.ts))
            self.upTargb3 = self.b3_target.assign(self.b3_target*(1-simple_actor_network.ts)+ self.b3*(simple_actor_network.ts))
            
            
    #        init = tf.initialize_variables([self.W1_target, self.W2_target, self.W3_target, self.b1_target, self.b2_target, self.b3_target])        
    #        
    #        self.sess.run(init)
            
            self.batch_state = []
            self.batch_actgrad = []
            
            self.aglomerate_params = tf.reshape(tf.concat(0,[tf.reshape(self.W1, [-1]), self.b1, tf.reshape(self.W2, [-1]), self.b2, tf.reshape(self.W3, [-1]), self.b3]) , [-1])
            self.param_loader = tf.placeholder(tf.float32, [state_size*l1_size + l1_size + l1_size*l2_size + l2_size + l2_size*action_size + action_size])
            
            self.param_loader_op = [self.W1.assign(tf.reshape(tf.slice(self.param_loader, [0], [state_size*l1_size]), [state_size, l1_size])),
                                    self.b1.assign(tf.slice(self.param_loader, [state_size*l1_size], [l1_size])),
                                    self.W2.assign(tf.reshape(tf.slice(self.param_loader, [state_size*l1_size+ l1_size], [l1_size*l2_size]), [l1_size, l2_size])),
                                    self.b2.assign(tf.slice(self.param_loader, [state_size*l1_size+ l1_size + l1_size*l2_size], [l2_size])),
                                    self.W3.assign(tf.reshape(tf.slice(self.param_loader, [state_size*l1_size+ l1_size + l1_size*l2_size+ l2_size], [l2_size*action_size]), [l2_size, action_size])),
                                    self.b3.assign(tf.slice(self.param_loader, [state_size*l1_size+ l1_size + l1_size*l2_size + l2_size + l2_size*action_size], [action_size]))]

    def action(self, state, target=False):
        """Return the actor's action for state"""
        if(target):
            return self.sess.run(self.action_output_target, feed_dict={self.state_input: [state]})[0]
        return self.sess.run(self.action_output, feed_dict={self.state_input: [state]})[0]
        
    def action_batch(self, state_batch, target=False):
        if(target):
            return self.sess.run(self.action_output_target, feed_dict={self.state_input: state_batch})
        return self.sess.run(self.action_output, feed_dict={self.state_input: state_batch})
        
    def learning_piece(self, state, act_grad):
        """add a training sample to the network's update, do not perform the update"""
        self.batch_state.append(state)
        self.batch_actgrad.append(act_grad)
    def batch(self, state, act_grad):
        self.batch_state = state
        self.batch_actgrad = act_grad
    def update(self):
        self.sess.run(self.updater, feed_dict={self.state_input: self.batch_state, self.action_gradient: self.batch_actgrad})
        #del self.batch_state[:]
        #del self.batch_actgrad[:]
    def updateTarget(self):
        self.sess.run([self.upTargW1,
                       self.upTargW2,
                       self.upTargW3,
                       self.upTargb1,
                       self.upTargb2,      
                       self.upTargb3])
    def linear_parameters(self):
        return self.sess.run(self.aglomerate_params)
    def load_parameters(self, parameters):
        self.sess.run(self.param_loader_op, feed_dict={self.param_loader : parameters})
        
        