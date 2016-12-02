function Qup = updateQ_add(yk,Qks,Q)

ndim = size(Q,1);

etak = [yk;2*Qks'];
betak = -Q*etak;

kappa = 2 - etak'*Q*etak;

Qup = [Q,zeros(ndim,1);zeros(1,ndim),0] + [betak;1]*[betak',1]/kappa;

