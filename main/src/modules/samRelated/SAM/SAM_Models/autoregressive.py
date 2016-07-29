# Andreas Damianou, 2016
# WYSIWYD

import numpy as np

def transformTimeSeriesToSeq(Y, timeWindow):
    Ntr,D = Y.shape
    blocksNumber = Ntr - timeWindow
    X = np.zeros((blocksNumber, timeWindow*D))
    Ynew = np.zeros((blocksNumber,D))
    for i in range(blocksNumber):
        tmp = Y[i:i+timeWindow,:].T
        X[i,:] = tmp.flatten().T
        Ynew[i,:] = Y[i+timeWindow,:]
    
    return X, Ynew


def transformSeqToTimeSeries(X, Y, timeWindow):
    assert(X.shape[0] == Y.shape[0])
    N = X.shape[0] + timeWindow
    D = X.shape[1] / (timeWindow * 1.0)
    Ynew = np.zeros((N, D))
    for i in range(X.shape[0]):
        Ynew[i:i+timeWindow, :] = X[i,:].reshape(D, timeWindow).T
    Ynew[-1,:] = Y[-1,:]
    return Ynew

def test_transformSeries(Y, timeWindow):
    (xx,yy) = transformTimeSeriesToSeq(Y, timeWindow)
    return transformSeqToTimeSeries(xx,yy,timeWindow)

def gp_narx(m, x_start, N, Uts, ws, Ydebug=None):
    # m is a GP model from GPy.

    D = m.output_dim
    Q = x_start.shape[1]
    Y = np.empty((N,D,))
    Y[:] = np.NAN
    varY = Y.copy()
    assert(Q%ws==0)
    assert(D == Q/ws)

    Xnew = m.X.copy()
    Ynew = m.Y.copy()


    curX = x_start

    varYpred = None

    for i in range(N):
        # Make sure the added x_add is a matrix (1,Q) and not (Q,)
        if len(curX.shape) < 2:
            curX = curX.reshape(1,curX.shape[0])
        varYpred_prev = varYpred
        #Ypred, varYpred = m._raw_predict(np.hstack((curX,curU)))
        #curU = Uts[i,:]
        #Ypred, varYpred = m._raw_predict(np.hstack((curX,Uts[i,:][None,:])))
        if Uts is not None:
            Ypred, varYpred = m.predict(np.hstack((curX,Uts[i,:][None,:])))
        else:
            Ypred, varYpred = m.predict(curX)

        Y[i,:] = Ypred
        varY[i,:] = varYpred

        #print i, ': ', Y[i,:] , ' | var: ', varYpred  #####

        if Ydebug is not None:
            if Uts is not None:
                print i, ': X=', str(curX.flatten()), 'U=', str(Uts[i,:].flatten()), 'Y=', str(Ydebug[i,:]) 
            else:
                print i, ': X=', str(curX.flatten()), 'U=None', 'Y=', str(Ydebug[i,:]) 

        if i == N-1:
            break


        curX = np.hstack((curX[0,D:], Ypred[0,:]))

    return Y, varY