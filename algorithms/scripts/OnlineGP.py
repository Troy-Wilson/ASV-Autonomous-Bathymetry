from __future__ import division
__author__ = 'troyu'

import numpy as np
import matplotlib.pyplot as plt
import CholFact
from scipy.optimize import fmin_l_bfgs_b
import time
from scipy.linalg.lapack import dpotrs, dpotri, dpotrf
from scipy import linalg
import threading
import scipy

# Class to conduct GP with a simple exponential covariance function effciently online using sequential
# cholesky updates

log2pi = np.log(2*np.pi)
epsilon = np.finfo(np.float64).resolution

class arrayManip():
    #some routines to get data in format for myGP to run and to convert it back to a grid format for plotting

    def createRegXStar2D(self, xmin, xmax, ymin, ymax,steps):
        #creates a regularly spaced array [steps*steps, 2] that can be fed into GP.predict as an
        #evenly space XStar array
        xdisc = float(xmax-xmin)/steps
        ydisc = float(ymax-ymin)/steps
        xs1 = np.arange(xmin,xmax, xdisc)*np.ones([steps,1])
        ys1 = np.arange(ymin,ymax, ydisc)*np.ones([steps,1])
        xs2 = xs1.reshape(steps*steps,1)
        ys2 = ys1.T.reshape(steps*steps,1)
        X = np.hstack([xs2,ys2])
        return X

    def convReg2DtoGrid(self, Xstar, YStar):
        #Converts the output of GP.predict() into a grid format for surface plotting
        # ie ax.plot_surface(X0G, X1G, YY, rstride=1, cstride=1, cmap=cm.cool)
        steps = np.sqrt(Xstar.shape[0])
        X0 = Xstar[:steps,0]
        X1 = Xstar[:,1].reshape([steps,steps])[:,0].T
        X0G, X1G = np.meshgrid(X0, X1)
        YY = YStar.reshape([steps,steps])
        return X0G, X1G, YY

class GP():
    # setData
    # setTheta
    # fit
    # predict
    # loop
    #   addFitObs
    #   predict

    def __init__(self,kernel="se"):
        self.X = []
        self.Y = []
        self.K = []
        self.K_star = []
        self.K_starstar = []
        self.X_star = []
        self.L = []
        self.var_f = None
        self.var_n = None
        self.charLen = None
        self.kernel = kernel
        ver = scipy.__version__.split('.')
        if int(ver[1]) <16:
            print '*******************'
            print '*******************'
            print '*******************'
            print '*******************'
            print 'please install scipy 0.16 or above'
            print 'GP.optimise may fail to find good solution on earlier versions'
            print '*******************'
            print '*******************'
            print '*******************'
            print '*******************'

    def makeMatrixCol(self,A):
        if not isinstance(A,np.ndarray):
            return np.array([[A]])
        if len(A.shape) == 1:
            return A[:, np.newaxis]
        else:
            return A

    def makeMatrixRow(self,A):
        if not isinstance(A,np.ndarray):
            return np.array([[A]])
        if len(A.shape) == 1:
            return A[np.newaxis,:]
        else:
            return A

    def setData(self,X, Y):
        lock = threading.RLock()
        with lock:
            self.X = self.makeMatrixCol(X)
            self.Y = self.makeMatrixCol(Y)

    def setThetaFitAll(self,X, Y, theta):
        #lock down until done so no other process/thread can access data mid calculation
        lock = threading.RLock()
        with lock:
            self.setData(X, Y)
            self.setTheta(theta)
            self.fit()

    def addFitObs(self, X, Y):
        #lock down until done so no other process/thread can access data mid calculation
        #this is for single obs, batch is a trivial addition, the batch function in CholFact is already done
        lock = threading.RLock()
        with lock:
            if self.X == []:
                self.setData(X, Y)
                self.fit()
            else:
                #add one observation and fit data
                X = self.makeMatrixCol(X)
                Y = self.makeMatrixCol(Y)
                K21 = self.cov(X1=X, X2=self.X)
                K22 = self.cov(X1=X) + self.var_n
                K1 = np.hstack([self.K, K21.T])
                K2 = np.hstack([K21,K22])
                LT = CholFact.updateObs(self.K,self.L.T,K2.T)
                self.L = LT.T
                #update K
                self.K = np.vstack([K1,K2])
                #stack obs on end of current data
                self.X = np.vstack([self.X, X])
                self.Y = np.vstack([self.Y,Y])

    def setTheta(self, theta):
        lock = threading.RLock()
        with lock:
            self.var_f = theta[0]
            self.charLen = theta[1]
            self.var_n = theta[2]

    def se_kern(self,X1, X2):
        r = self.ed(X1, X2)/self.charLen
        K = self.var_f*np.exp(-0.5*r**2)
        return K, r

    def ed(self, X1, X2):
        #return Euclidian Distance
        #now check if X1 and X2 are vectors or arrays
        if X1.shape[1] == 1: #vectors
            n = X1.shape[0]
            m = X2.shape[0]
            r1 = X1.reshape(n,1)*np.ones([1,m])
            r2 = X2.reshape(1,m)*np.ones([n,1])
            ed =  np.sqrt((r1-r2)**2)
        elif X1.shape[1] == 2: #matricies
            n = X1.shape[0]
            m = X2.shape[0]
            r1x = X1[:,0].reshape(n,1)*np.ones([1,m])
            r1y = X1[:,1].reshape(n,1)*np.ones([1,m])
            r2x = X2[:,0].reshape(1,m)*np.ones([n,1])
            r2y = X2[:,1].reshape(1,m)*np.ones([n,1])
            ed =  np.sqrt((r1x-r2x)**2+(r1y-r2y)**2)
        else:
            print "too many dimensions in X matricies" , X1.shape
            return None
        return ed

    def cov(self,X1, X2 = [], retr = False):
        if X2 == []:
            X2 = X1
        if self.kernel == 'se':
            K, r = self.se_kern(X1,X2)
        else:
            print "plese select valid Kernel type: se"
            K, r = None, None
        if retr == False:
            return K
        else:
            return K, r

    def setK_star(self):
        lock = threading.RLock()
        with lock:
            self.K_star = self.cov(X1=self.X_star,X2=self.X)

    def fit(self):
        lock = threading.RLock()
        with lock:
            self.K = self.cov(X1 = self.X) + np.eye(self.X.shape[0])*self.var_n
            self.L = self.cholSafe(self.K)

    def pred(self,X_star, varinfo=False):
        lock = threading.RLock()
        with lock:
            self.X_star = self.makeMatrixCol(X_star)
            self.setK_star()
            alpha = np.linalg.solve(self.L.T,np.linalg.solve(self.L,self.Y))
            y_star = np.dot(self.K_star,alpha)
            if varinfo == False:
                return y_star
            else:
                self.K_starstar = self.cov(X1 = self.X_star)
                vega = np.linalg.solve(self.L, self.K_star.T)
                y_star_var = self.K_starstar-np.dot(vega.T, vega)
                return y_star, y_star_var

    def entropy(self):
        #entropy in nats (for bits use np.log2())
        K1 = np.hstack([self.K, self.K_star.T])
        K2 = np.hstack([self.K_star,self.K_starstar])
        K_new = np.vstack([K1,K2])
        K_new_det = np.linalg.det(K_new)
        nats = 0.5*np.log(K_new_det)
        return nats

    def genY(self,X,theta):
        self.setTheta(theta)
        C = self.cov(X) + np.eye(self.X.shape[0])*self.var_n
        mu = 0.
        Y = np.random.multivariate_normal(mu,C,1).T
        return Y

    def LML(self,theta,returnGradients=False):
        if self.kernel == 'se':
            return self.LML_se(theta,returnGradients=returnGradients)
        else:
            print "plese select valid Kernel type: se"

    def LML_se(self,theta,returnGradients=False):
        self.setTheta(theta)
        K,r = self.cov(self.X,retr=True)
        Ky = K.copy()
        Ky +=  np.eye(self.X.shape[0])*self.var_n + np.eye(self.X.shape[0])*1e-8
        L = self.cholSafe(Ky)
        WlogDet = 2.*np.sum(np.log(np.diag(L)))
        alpha, status = dpotrs(L, self.Y, lower=1)
        dataFit = - np.sum(alpha * self.Y)
        modelComplexity = -self.Y.shape[1] * WlogDet
        normalizer = -self.Y.size * log2pi
        logMarginalLikelihood = 0.5*(dataFit + modelComplexity + normalizer)
        if returnGradients == False:
            return logMarginalLikelihood
        else:
            Wi, status = dpotri(-L, lower=1)
            Wi = np.asarray(Wi)
            # copy bottom triangle to top triangle
            triu = np.triu_indices_from(Wi,k=1)
            Wi[triu] = Wi.T[triu]
            # dL = change in LML, dK is change in Kernel(K)
            dL_dK = 0.5 * (np.dot(alpha,alpha.T) - self.Y.shape[1] * Wi)
            dL_dVarn = np.diag(dL_dK).sum()
            varfGradient = np.sum(K* dL_dK)/self.var_f
            dK_dr = -r*K
            dL_dr = dK_dr * dL_dK
            lengthscaleGradient = -np.sum(dL_dr*r)/self.charLen
            grads = np.array([varfGradient, lengthscaleGradient, dL_dVarn])
            return logMarginalLikelihood, grads

    def Neg_LML_Grads(self,theta):
        LML, grads = self.LML(theta, returnGradients=True)
        hp = np.array([self.var_f,self.charLen,self.var_n])
        trans_neg_grads = self.transformGrads(hp,-grads)
        return -LML, trans_neg_grads
        #return -LML, -grads

    def transformGrads(self,hp, dhp):
        #run a sigmoid function over the weights.
        weights = (1/(1+np.exp(-hp)))#1-(1/(1+np.exp(hp)))
        ret = weights*dhp
        return ret

    def cholSafe(self, A):
        # if lapack cholesky decomp fails then consecutively add more jitter until we get
        # a working cholesky (ie ensuring we have a positive definite matrix as it can sometimes
        # due to floating point errors end up not positive definite).
        A = np.ascontiguousarray(A)
        L, status = dpotrf(A, lower=1)
        if status == 0:
            return L
        else:
            L = linalg.cholesky(A + np.eye(A.shape[0]) * 1e-5, lower=True)
        return L

    def optimise(self, theta=np.array([1.,1.,1.]), bounds=[(1e-8,None),(1e-8,None),(1e-8,None)],
                 messages=False):
        res = fmin_l_bfgs_b(self.Neg_LML_Grads,theta, epsilon=1e-08, factr=1e7, bounds = bounds,
                      maxiter=15000, maxfun=1000,pgtol=1e-05,m=10)
        if messages == True:
            print res[2]
        return res[0],res[1]

    def optimiseRandStart(self,num=10,messages=False, bounds=[(1e-8,None),(1e-8,None),(1e-8,None)]):
        bestnegLML = 99999999999999999
        bestTheta = None
        for i in range(num):
            timer = time.time()
            theta = np.random.exponential(1.0,3)
            for j in range(theta.shape[0]):
                if bounds[j][0] != None:
                    theta[j] = max(theta[j],bounds[j][0])
                if bounds[j][1] != None:
                    theta[j] = min(theta[j],bounds[j][1])
            res = self.optimise(theta=theta, bounds=bounds)
            if res[1] < bestnegLML:
                bestnegLML = res[1]
                bestTheta = res[0]
            if messages == True:
                print "thetares",  '{:.6f}'.format(res[0][0]), ", ",\
                '{:.6f}'.format(res[0][1]), ", ",\
                '{:.6f}'.format(res[0][2]), \
                "LML",  '{:.6f}'.format(-res[1]),\
                "thetaStart", np.around(theta,3) , "secs", time.time() - timer
        return bestTheta, -bestnegLML
