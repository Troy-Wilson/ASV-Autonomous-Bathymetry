#!/usr/bin/env python
__author__ = 'troy'

import numpy as np

class Kalman():
    def __init__(self, XhatDim, Xhat, P, Q):
        self.XhatDim = XhatDim
        self.Xhat = Xhat
        self.P = P
        self.Q = Q

    def pred(self, F):
        self.Xhat = np.dot(F, self.Xhat)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update(self, z, H, R):
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(np.dot(np.dot(H, self.P), H.T) + R))
        self.Xhat = self.Xhat + np.dot(K, (z-np.dot(H, self.Xhat)))
        self.P = np.dot((np.eye(self.XhatDim)-np.dot(K, H)), self.P)

class Kalmandt(): #version with variable predict/update time intervals
    def __init__(self, Xhat, P):
        self.XhatDim = Xhat.shape[0]
        self.Xhat = Xhat
        self.P = P

    def pred(self, F, Q):
        self.Xhat = np.dot(F, self.Xhat)
        self.P = np.dot(np.dot(F, self.P), F.T) + Q

    def update(self, z, H, R):
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(np.dot(np.dot(H, self.P), H.T) + R))
        self.Xhat = self.Xhat + np.dot(K, (z-np.dot(H, self.Xhat)))
        self.P = np.dot((np.eye(self.XhatDim)-np.dot(K, H)), self.P)

