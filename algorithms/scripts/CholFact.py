__author__ = 'troy'

import numpy as np
import numpy.linalg as la

# note update is numerically stable, downdate is not(ref linpack manual) .
# There are minor diferences due to floating point errors
# Only implemented update
# Adjusted from algorithm in Osbourne2008, A.1, for case where data added is on the end
# his algorithm was more complex for adding data in the middle
# Original psd matrix V is nxn
# Original Upper triangular Cholesky decomposition LStar is also nxn
# Vnew and LStarNew are (n+1)x(n+1)
# Vcnew = np.vstack([V12,V22])
#
# So we have old matricies for V and LStar:
# [V11] [R11]
# and new matricies for Vnew and LStarNew:
#
# |V11   V12| |S11 S12|
# |V12.T V22| |0   S22|
#
# Following proof in Osbourne in solving for A triangular of x = A\b we get
#
# S11 = R11
# S12 = R11.T\V12             in python S12 = la.solve(LStar.T,Vcn1)
# S22 = Chol(V22 - S12.T*S12) in python S22 = la.cholesky(Vcn2-np.dot(s12.T,s12))

# to add 100 obs to a list fo 500 obs, the batch update takes 0.21s, whereas the full Cholesky update takes 0.92s

def updateObs(V, LStar, Vcnew):
    # takes in original V and Lstar matricies of size nxn with Lstar upper triangular
    # Vrnew is the new row of V which is n+1 x 1
    # returns LStarNew, the updated Cholesky upper triangular matrix
    # this is thus adding 1 new observation to the psd (correlation) matrix V
    szV = len(V)
    szVn = Vcnew.shape[0]
    if szV != szVn-1:
        print 'error, Vrnew is not 1 column longer than V'
        LStarNew = None
        return LStarNew
    #add zeros to bottom of LStar for LStarNew
    LStarNew = np.vstack([LStar,np.zeros([1,szV])])
    Vcn1 = Vcnew[:-1,:]
    Vcn2 = Vcnew[-1,:]
    s12 = la.solve(LStar.T,Vcn1)
    v22 = Vcn2-np.dot(s12.T,s12)
    # use cholesky here if a matrix
    s22s = np.sqrt(v22)
    s2 = np.vstack([s12,s22s])
    LStarNew = np.hstack([LStarNew,s2])
    return LStarNew

def updateBatch(V, LStar, Vcnew):
    # takes in original V and Lstar matricies of size nxn with Lstar upper triangular
    # Vrnew is the new matrix of V which is n+m x m
    # returns LStarNew, the updated Cholesky upper triangular matrix
    # this is thus adding 1 new observation to the psd (correlation) matrix V
    n = V.shape[0]
    n_m = Vcnew.shape[0]
    m = n_m - n
    if m <=0:
        print 'error, Vrnew is not longer than V'
        LStarNew = None
        return LStarNew
    #add zeros to bottom of LStar for LStarNew
    LStarNew = np.vstack([LStar,np.zeros([m,n])])
    Vcn1 = Vcnew[:n,:]
    Vcn2 = Vcnew[n:n_m,:]
    s12 = la.solve(LStar.T,Vcn1)
    s22m = la.cholesky(Vcn2-np.dot(s12.T,s12)).T
    s2 = np.vstack([s12,s22m])
    LStarNew = np.hstack([LStarNew,s2])
    return LStarNew
