import SAM
import GPy

from SAM.SAM_Models import autoregressive
import pylab as pb
import numpy as np

num_inducing = 40

# Create toy data
N1 = 140
Ntr = np.ceil(N1/2)
X1 = np.random.rand(N1, 1) * 8*np.pi
X1.sort(0)
Y1 = np.sin(X1) + np.random.randn(*X1.shape) * 0.02
#U1 = X1%(2*np.pi)


#----------- Train a standard model
m1=SAM.SAM_Core.LFM()
m1.store(observed={'Y':Y1[0:Ntr,:]}, inputs=X1[0:Ntr,:], Q=None, kernel=None, num_inducing=num_inducing)
m1.learn()
ret = m1.visualise()
y_pred_standard = m1.pattern_completion(X1[Ntr:,:])[0]
pb.figure()
pb.plot(X1[Ntr:,:],Y1[Ntr:,:], 'x-')
pb.plot(X1[Ntr:,:],y_pred_standard, 'ro-')
pb.legend(('True','Pred'))
pb.title('Standard GP')
#---------------------------------------


# Create transformed data (autoregressive dataset)
ws=10 # Windowsize
xx,yy = autoregressive.transformTimeSeriesToSeq(Y1, ws)

#uu,tmp = transformTimeSeriesToSeq(U1, ws)
# Test the above: np.sin(uu) - xx

#uu = yy**2 -2*yy + 5 + np.random.randn(*yy.shape) * 0.005
U1 = Y1**2 -2*Y1 + 5 + np.random.randn(*Y1.shape) * 0.005
uu,tmp = autoregressive.transformTimeSeriesToSeq(U1, ws)

Xtr = xx[0:Ntr,:]
Xts = xx[Ntr:,:]
Ytr = yy[0:Ntr,:]
Yts = yy[Ntr:,:]
Utr = uu[0:Ntr,:]
Uts = uu[Ntr:,:]


#----------  Train autoregressive model with additional (exogenous) inputs
m_autoreg = GPy.models.SparseGPRegression(np.hstack((Xtr,Utr)),Ytr, num_inducing = num_inducing)
m_autoreg.optimize('bfgs', max_iters=1000, messages=True)
print m_autoreg


# Initial window to kick-off free simulation
x_start = Xts[0,:][:,None].T

# Free simulation
ygp, varygp = autoregressive.gp_narx(m_autoreg, x_start, Yts.shape[0], Uts, ws)
pb.figure()
pb.plot(Yts, 'x-')
pb.plot(ygp, 'ro-')
pb.legend(('True','Pred'))
pb.title('NARX-with-exogenous')
#--------------------------------------------






# #----------  Train autoregressive model with no (exogenous) inputs
m_autoreg2 = GPy.models.SparseGPRegression(Xtr,Ytr, num_inducing = num_inducing)
print m_autoreg2

# Initial window to kick-off free simulation
x_start = Xts[0,:][:,None].T

# Free simulation
ygp2, varygp2 = autoregressive.gp_narx(m_autoreg2, x_start, Yts.shape[0], None, ws)
pb.figure()
pb.plot(Yts, 'x-')
pb.plot(ygp2, 'ro-')
pb.legend(('True','Pred'))
pb.title('NARX-no-exogenous')
#--------------------------------------------
