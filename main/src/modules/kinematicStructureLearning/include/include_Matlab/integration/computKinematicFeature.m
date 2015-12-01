function [feat1,feat2] = computKinematicFeature(KineStruct_P, KineStruct_Q, t1)
nNode1 = KineStruct_P.num_seg;
nNode2 = KineStruct_Q.num_seg;

aff1 = KineStruct_P.affinity;
aff2 = KineStruct_Q.affinity;

feat1 = zeros(1,length(t1));
feat2 = zeros(1,nNode2^2);

t1 = t1 + 1;

%% Cal feat1
for t=1:length(t1)
    i= t1(1,t);
    j= t1(2,t);
    
    if i==j
        feat1(t) = -10;
    else
        feat1(t) = aff1(i,j);
    end
end

%% Cal feat2
t=0;
for i=1:nNode2
    for j=1:nNode2
        t = t+1;
        
        if i==j
            feat2(t) = -10;
        else
            feat2(t) = aff2(i,j);
        end
    end
end
end
