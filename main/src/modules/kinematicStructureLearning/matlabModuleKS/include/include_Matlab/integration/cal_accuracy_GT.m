function accuracy = cal_accuracy_GT(KineStruct_P, KineStruct_Q, X)
%%
fileName_P = KineStruct_P.videoFileName;
fileName_Q = KineStruct_Q.videoFileName;

num_node = KineStruct_P.num_seg;
X_gt = zeros(KineStruct_P.num_seg,KineStruct_Q.num_seg);

pairName = [fileName_P(1:end-4),'-',fileName_Q(1:end-4)];
%%
switch(pairName)
    case 'dancing-dancing_iCub'
        X_gt(1,6) = 1;
        X_gt(2,1) = 1;
        X_gt(3,2) = 1;
        X_gt(4,5) = 1;
        X_gt(5,4) = 1;
        X_gt(6,3) = 1;
%         X_gt(2,5) = 1;
%         X_gt(3,4) = 1;
%         X_gt(4,1) = 1;
%         X_gt(5,2) = 1;
    case 'dancing-dancing_Nao'
        X_gt(1,1) = 1;
        X_gt(2,9) = 1;
        X_gt(3,8) = 1;
        X_gt(4,11) = 1;
        X_gt(5,10) = 1;
        X_gt(6,2) = 1;
        X_gt(2,11) = 1;
        X_gt(3,10) = 1;
        X_gt(4,9) = 1;
        X_gt(5,8) = 1;
    case 'dancing_iCub-dancing_Nao'
        X_gt(1,9) = 1;
        X_gt(2,8) = 1;
        X_gt(3,2) = 1;
        X_gt(4,10) = 1;
        X_gt(5,11) = 1;
        X_gt(6,1) = 1;
        X_gt(7,3) = 1;
        X_gt(1,11) = 1;
        X_gt(2,10) = 1;
        X_gt(4,8) = 1;
        X_gt(5,9) = 1;        
    case 'dancing-puppet_human'
        X_gt(1,1) = 1;
        X_gt(2,4) = 1;
%         X_gt(2,6) = 1;
        X_gt(3,3) = 1;
%         X_gt(3,5) = 1;
        X_gt(4,6) = 1;
%         X_gt(4,4) = 1;
        X_gt(5,5) = 1;
        X_gt(6,2) = 1;
    case 'digger_Baxter-robot_arm'
        X_gt(5,1) = 1;
        X_gt(4,2) = 1;
        X_gt(3,3) = 1;
        X_gt(2,4) = 1;
        X_gt(1,5) = 1;
    case 'iCub_body-dancing_iCub'
        X_gt(6,6) = 1;
        X_gt(3,3) = 1;
        X_gt(7,7) = 1;
        X_gt(7,6) = 1;
        X_gt(6,7) = 1;
        X_gt(1,1) = 1;
        X_gt(2,2) = 1; 
        X_gt(4,4) = 1;
        X_gt(5,5) = 1;
        X_gt(1,5) = 1;
        X_gt(2,4) = 1;
        X_gt(4,2) = 1;
        X_gt(5,1) = 1;      
    case 'puppet-puppet_human'
        X_gt(1,1) = 1;
        X_gt(2,2) = 1;
        X_gt(3,3) = 1;
        X_gt(3,5) = 1;
        X_gt(4,3) = 1;
        X_gt(4,5) = 1;
        X_gt(5,7) = 1;
        X_gt(6,8) = 1;
        X_gt(7,10) = 1;       
    case 'puppet-puppet_Nao'
        X_gt(1,1) = 1;
        X_gt(2,2) = 1;
        X_gt(3,8) = 1;
        X_gt(4,10) = 1;
        X_gt(3,10) = 1;
        X_gt(4,8) = 1;
        X_gt(5,3) = 1;
        X_gt(6,4) = 1;
        X_gt(7,6) = 1;           
        X_gt(6,6) = 1;
        X_gt(7,4) = 1; 
    case 'yellow_crane-digger_arm_left'
        X_gt(1,5) = 1;
        X_gt(2,4) = 1;
        X_gt(3,3) = 1;
        X_gt(4,2) = 1;
        X_gt(4,1) = 1;
    case 'yellow_crane-digger_finger_left'
        X_gt(1,4) = 1;
        X_gt(2,3) = 1;
        X_gt(3,2) = 1;
        X_gt(4,1) = 1;
    case 'iCub_body-puppet_Nao'
        X_gt(6,1) = 1;
        X_gt(1,9) = 1;
        X_gt(1,11) = 1;
        X_gt(2,8) = 1;  
        X_gt(2,10) = 1;
        X_gt(3,2) = 1;
        X_gt(4,8) = 1;
        X_gt(4,10) = 1;      
        X_gt(5,9) = 1;      
        X_gt(5,11) = 1;      
        X_gt(7,3) = 1;      
    case 'digger_finger_left-digger_arm_left'      
        X_gt(1,1) = 1;
        X_gt(2,2) = 1;
        X_gt(2,3) = 1;
        X_gt(3,3) = 1;
        X_gt(3,4) = 1;
        X_gt(4,5) = 1;
end

%%
accuracy = nnz(X .* X_gt) / num_node * 100;

end