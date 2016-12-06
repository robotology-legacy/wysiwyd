function Qup = updateQ_removal(Q,idx)

idx = idx+1;

rem_idx = [1:idx-1,idx+1:size(Q,1)];
Qup = Q(rem_idx,rem_idx) - Q(rem_idx,idx)*Q(idx,rem_idx)/Q(idx,idx);

if Qup == -inf
    Qup = 0;
end
    