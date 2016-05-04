# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 14:44:14 2016

@author: debroissia
"""
from critic_network import critic_network
import math

import tensorflow as tf

from helper.weight_norm import weight_norm

class simple_critic_network(critic_network):
    """A first critic network for low-dim state"""
    l1_size = 400
    l2_size = 300
    learning_rate = 0.0001
    ts = 0.001
    def __init__(self, state_size, action_size, action_bound = None):
        l1_size = simple_critic_network.l1_size
        l2_size = simple_critic_network.l2_size
        
        self.graph = tf.Graph()
        with self.graph.as_default():
            self.sess = tf.Session()

            
            self.state_input = tf.placeholder(tf.float32, [None, state_size])
            self.action_input = tf.placeholder(tf.float32, [None, action_size])
            #self.action_input_1d = tf.placeholder(tf.float32, [action_size])
    
            self.W1 = weight_norm(state_size, l1_size, [-1/math.sqrt(state_size), 1/math.sqrt(state_size)], self.graph).w
            self.W2 = weight_norm(l1_size, l2_size,[-1/math.sqrt(l1_size+action_size), 1/math.sqrt(l1_size+action_size)], self.graph).w
            self.W2_action = weight_norm(action_size, l2_size,[-1/math.sqrt(l1_size+action_size), 1/math.sqrt(l1_size+action_size)], self.graph).w
            self.W3 = weight_norm(l2_size, 1, [-0.0003, 0.0003], self.graph).w
    
            self.b1 = tf.Variable(tf.random_uniform([l1_size], -1/math.sqrt(state_size), 1/math.sqrt(state_size)))
            self.b2 = tf.Variable(tf.random_uniform([l2_size], -1/math.sqrt(l1_size+action_size), 1/math.sqrt(l1_size+action_size)))
            self.b3 = tf.Variable(tf.random_uniform([1], -0.0003, 0.0003))
    
            self.W1_target = tf.Variable(tf.zeros([state_size, l1_size]))
            self.W2_target = tf.Variable(tf.zeros([l1_size, l2_size]))
            self.W2_action_target = tf.Variable(tf.zeros([action_size, l2_size]))
            self.W3_target = tf.Variable(tf.zeros([l2_size, 1]))
            
            self.b1_target = tf.Variable(tf.zeros([l1_size]))
            self.b2_target = tf.Variable(tf.zeros([l2_size]))
            self.b3_target = tf.Variable(tf.zeros([1]))
            
            self.x1 = tf.nn.softplus(tf.matmul(self.state_input,self.W1) + self.b1)
            self.x2 = tf.nn.softplus(tf.matmul(self.x1,self.W2) + tf.matmul(self.action_input,self.W2_action) + self.b2)
            self.qval_output = tf.matmul(self.x2,self.W3) + self.b3
    
            self.x1_target = tf.nn.softplus(tf.matmul(self.state_input,self.W1_target) + self.b1_target)
            self.x2_target = tf.nn.softplus(tf.matmul(self.x1_target,self.W2_target) + tf.matmul(self.action_input,self.W2_action_target) + self.b2_target)
            self.qval_output_target = tf.matmul(self.x2_target,self.W3_target) + self.b3_target
    
            self.act_grad_v = tf.gradients(self.qval_output, self.action_input)
            self.act_grad = [self.act_grad_v[0]/tf.to_float(tf.shape(self.act_grad_v[0])[0])]
            
            
            self.qval_train = tf.placeholder(tf.float32, [None, 1])
            self.diff =tf.pow(self.qval_output-self.qval_train, 2)/tf.to_float(tf.shape(self.qval_train)[0]) + 0.01*tf.reduce_sum(tf.pow(self.W2,2))+ 0.01*tf.reduce_sum(tf.pow(self.b2,2))
            #self.params = [self.W1, self.W2, self.W2_action, self.W3, self.b1, self.b2, self.b3]
            #self.params_grad = tf.gradients(self.diff, self.params)
            
            self.adam = tf.train.AdamOptimizer(simple_critic_network.learning_rate)       
            self.optimizer = self.adam.minimize(self.diff)
            
            init = tf.initialize_all_variables()
            self.sess.run(init)
            
    
            self.sess.run([self.W1_target.assign(self.W1),
                           self.W2_target.assign(self.W2),
                           self.W2_action_target.assign(self.W2_action),
                           self.W3_target.assign(self.W3),
                           self.b1_target.assign(self.b1),
                           self.b2_target.assign(self.b2),
                           self.b3_target.assign(self.b3) ])  
            
            #self.optimizer = tf.train.GradientDescentOptimizer(simple_critic_network.learning_rate).minimize(self.diff)
            #self.updater = self.optimizer.apply_gradients(zip(self.params_grad, self.params))        
            
            self.upTargW1 =self.W1_target.assign(self.W1_target*(1-simple_critic_network.ts)+ self.W1*(simple_critic_network.ts))        
            self.upTargW2 =self.W2_target.assign(self.W2_target*(1-simple_critic_network.ts)+ self.W2*(simple_critic_network.ts))
            self.upTargW2a =self.W2_action_target.assign(self.W2_action_target*(1-simple_critic_network.ts)+ self.W2_action*(simple_critic_network.ts))
            self.upTargW3 =self.W3_target.assign(self.W3_target*(1-simple_critic_network.ts)+ self.W3*(simple_critic_network.ts))
            
            self.upTargb1 =self.b1_target.assign(self.b1_target*(1-simple_critic_network.ts)+ self.b1*(simple_critic_network.ts))
            self.upTargb2 =self.b2_target.assign(self.b2_target*(1-simple_critic_network.ts)+ self.b2*(simple_critic_network.ts))
            self.upTargb3 =self.b3_target.assign(self.b3_target*(1-simple_critic_network.ts)+ self.b3*(simple_critic_network.ts))
            
    #        init = tf.initialize_variables([self.W1_target, self.W2_target, self.W2_action_target, self.W3_target, self.b1_target, self.b2_target, self.b3_target])        
    #        
    #        self.sess.run(init)
            
            self.batch_state = []
            self.batch_action = []
            self.batch_val = []
            
            
            self.gamma = 0.99
            self.rewards = tf.placeholder(tf.float32, [None, 1])
            self.q_vals_batch = tf.placeholder(tf.float32, [None, 1])
            self.y_opp = self.rewards + self.q_vals_batch*self.gamma
    def q_val(self, state, action, target=False):
        """return the estimated q value of the couple state-action"""
        if(target):
            return self.sess.run(self.qval_output_target, feed_dict={self.state_input: [state], self.action_input: [action]})[0][0]
        return self.sess.run(self.qval_output, feed_dict={self.state_input: [state], self.action_input: [action]})[0][0]
    def q_val_batch(self, state, action, target=False):
        if(target):
            return self.sess.run(self.qval_output_target, feed_dict={self.state_input: state, self.action_input: action})
        return self.sess.run(self.qval_output, feed_dict={self.state_input: state, self.action_input: action})
    def y_val_calc(self, reward, q_vals):
        return self.sess.run(self.y_opp, feed_dict={self.rewards: reward, self.q_vals_batch: q_vals})
    def actionGradient(self, state, action):
        """return the gradient of the action"""
        #print self.sess.run(self.grad_inverter, feed_dict={self.act_grad_placeholder: self.sess.run(self.act_grad, feed_dict={self.state_input: [state], self.action_input: [action]})[0][0], self.action_input_1d: [action]})
        return self.sess.run(self.act_grad, feed_dict={self.state_input: [state], self.action_input: [action]})[0][0]
    def actionGradient_batch(self, state, action):
        return self.sess.run(self.act_grad, feed_dict={self.state_input: state, self.action_input: action})[0]
    def learning_piece(self, state, action, value):
        """add a training sample to the network's update, do not perform the update"""
        self.batch_state.append(state)
        self.batch_action.append(action)
        self.batch_val.append([value])
    def batch(self, state, action, value):
        self.batch_state = state
        self.batch_action = action
        self.batch_val = value
    def update(self):
        self.sess.run(self.optimizer, feed_dict={self.state_input: self.batch_state, self.action_input: self.batch_action, self.qval_train: self.batch_val})
        #del self.batch_state[:]
        #del self.batch_action[:]
        #del self.batch_val[:]
    def updateTarget(self):
        self.sess.run([self.upTargW1,
                       self.upTargW2,
                       self.upTargW2a,
                       self.upTargW3,
                       self.upTargb1,
                       self.upTargb2,      
                       self.upTargb3])