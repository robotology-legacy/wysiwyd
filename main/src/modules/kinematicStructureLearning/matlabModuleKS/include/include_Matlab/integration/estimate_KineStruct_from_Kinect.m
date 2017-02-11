function KineStruct = estimate_KineStruct_from_Kinect(folderPath)
%%
% Load file from the filepath
disp('=========================================================================');
disp(['Perform Kinematic Structure Building from Kinect data...',folderPath]);
disp('=========================================================================');

edge_file = [folderPath,'/edges.txt'];
joint_file = [folderPath,'/joints.txt'];
node_file = [folderPath,'/nodes.txt'];

edge_info = importdata(edge_file);
joint_info = importdata(joint_file);
node_info = importdata(node_file,'-');

num_frames = length(unique(joint_info(:,1)));
num_seg = length(unique(joint_info(:,2)));
num_edges = length(edge_info);

height = 700;
width = 700;

joints = cell(num_frames,1);

for frm_idx = 1:num_frames
    joints{frm_idx} = joint_info((frm_idx-1)*num_seg+1:frm_idx*num_seg,6:7);
end

%% Draw
% figure(739)
% for frm_idx = 1:num_frames
%     clf
%     hold on
%     for node_idx = 1:num_seg
%         plot(joints{frm_idx}(node_idx,1), joints{frm_idx}(node_idx,2),'ro');
%         text(joints{frm_idx}(node_idx,1)+10, joints{frm_idx}(node_idx,2)-10, node_info{node_idx}, 'Color', 'b');
%     end
%
%     for edge_idx = 1:num_edges
%         plot([joints{frm_idx}(edge_info(edge_idx,1),1),joints{frm_idx}(edge_info(edge_idx,2),1)],...
%              [joints{frm_idx}(edge_info(edge_idx,1),2),joints{frm_idx}(edge_info(edge_idx,2),2)],'k-');
%     end
%
%     axis([-100,600,-100,600])
%     pause(0.5);
% end

%% New Nodes & Joints
numNode = 11;
numFrame = num_frames;

nodeLoc = zeros(2, numNode, numFrame);
joint_center_Kinect = cell(numNode, numNode);
for i = 1:num_seg
    for j = 1:num_seg
        if i~=j
            buf = zeros(2,numFrame);
            for frm_idx = 1:numFrame
                buf(1,frm_idx) = round((joints{frm_idx}(i,1) + joints{frm_idx}(j,1)) / 2);
                buf(2,frm_idx) = round((joints{frm_idx}(i,2) + joints{frm_idx}(j,2)) / 2);
            end
            joint_center_Kinect{i,j} = buf;
        end
    end
end

for frm_idx = 1 : numFrame
    nodeLoc(:,1,frm_idx) = joints{frm_idx}(1,:)';
    nodeLoc(:,2,frm_idx) = joints{frm_idx}(8,:)';
    nodeLoc(:,7,frm_idx) = joints{frm_idx}(15,:)';
    nodeLoc(:,4,frm_idx) = joints{frm_idx}(11,:)';
    nodeLoc(:,6,frm_idx) = joints{frm_idx}(4,:)';
    
    nodeLoc(:,3,frm_idx) = joint_center_Kinect{9,14}(:,frm_idx)';
    %     nodeLoc(:,4,frm_idx) = joint_center_Kinect{9,11}(:,frm_idx)';
    nodeLoc(:,5,frm_idx) = joint_center_Kinect{2,7}(:,frm_idx)';
    %     nodeLoc(:,6,frm_idx) = joint_center_Kinect{2,4}(:,frm_idx)';
    nodeLoc(:,8,frm_idx) = joint_center_Kinect{12,13}(:,frm_idx)';
    nodeLoc(:,9,frm_idx) = joint_center_Kinect{10,13}(:,frm_idx)';
    nodeLoc(:,10,frm_idx) = joint_center_Kinect{5,6}(:,frm_idx)';
    nodeLoc(:,11,frm_idx) = joint_center_Kinect{3,6}(:,frm_idx)';
end

joint_center = cell(numNode, numNode);
for i = 1:numNode
    for j = 1:numNode
        if i~=j
            buf = zeros(2,numFrame);
            for frm_idx = 1:numFrame
                buf(1,frm_idx) = round((nodeLoc(1,i,frm_idx) + nodeLoc(1,j,frm_idx)) / 2);
                buf(2,frm_idx) = round((nodeLoc(2,i,frm_idx) + nodeLoc(2,j,frm_idx)) / 2);
            end
            joint_center{i,j} = buf;
        end
    end
end

%% Joint assignment by hand
% joint_center = cell(numNode, numNode);

i=1;
j=2;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = round((nodeLoc(1,i,frm_idx) + nodeLoc(1,j,frm_idx)) / 2);
    buf(2,frm_idx) = round((nodeLoc(2,i,frm_idx) + nodeLoc(2,j,frm_idx)) / 2);
end
joint_center{i,j} = buf;

i=2;
j=7;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = round((nodeLoc(1,i,frm_idx) + nodeLoc(1,j,frm_idx)) / 2);
    buf(2,frm_idx) = round((nodeLoc(2,i,frm_idx) + nodeLoc(2,j,frm_idx)) / 2);
end
joint_center{i,j} = buf;

i=2;
j=3;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(14,1);
    buf(2,frm_idx) = joints{frm_idx}(14,2);
end
joint_center{i,j} = buf;

i=3;
j=4;
kinect_jnt_idx = 9;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=2;
j=5;
kinect_jnt_idx = 7;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=5;
j=6;
kinect_jnt_idx = 2;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=7;
j=8;
kinect_jnt_idx = 12;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=8;
j=9;
kinect_jnt_idx = 13;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=7;
j=10;
kinect_jnt_idx = 5;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=10;
j=11;
kinect_jnt_idx = 6;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

%----------------------------------
i=2;
j=1;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = round((nodeLoc(1,i,frm_idx) + nodeLoc(1,j,frm_idx)) / 2);
    buf(2,frm_idx) = round((nodeLoc(2,i,frm_idx) + nodeLoc(2,j,frm_idx)) / 2);
end
joint_center{i,j} = buf;

i=7;
j=2;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = round((nodeLoc(1,i,frm_idx) + nodeLoc(1,j,frm_idx)) / 2);
    buf(2,frm_idx) = round((nodeLoc(2,i,frm_idx) + nodeLoc(2,j,frm_idx)) / 2);
end
joint_center{i,j} = buf;

i=3;
j=2;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(14,1);
    buf(2,frm_idx) = joints{frm_idx}(14,2);
end
joint_center{i,j} = buf;

i=4;
j=3;
kinect_jnt_idx = 9;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=5;
j=2;
kinect_jnt_idx = 7;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=6;
j=5;
kinect_jnt_idx = 2;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=8;
j=7;
kinect_jnt_idx = 12;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=9;
j=8;
kinect_jnt_idx = 13;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=10;
j=7;
kinect_jnt_idx = 5;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;

i=11;
j=10;
kinect_jnt_idx = 6;
buf = zeros(2,numFrame);
for frm_idx = 1:numFrame
    buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
    buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
end
joint_center{i,j} = buf;


%%
connection = zeros(numNode, numNode);
connection(1,2) = 1;
connection(2,3) = 1;
connection(3,4) = 1;
connection(2,5) = 1;
connection(5,6) = 1;
connection(2,7) = 1;
connection(7,8) = 1;
connection(8,9) = 1;
connection(7,10) = 1;
connection(10,11) = 1;

ST = sparse(connection);
[idx_i, idx_j, value] = find(ST);
%---------------------------------------------------
figure(705)

for frm_idx = 1:numFrame
    clf
    hold on
    plot(nodeLoc(1,:,frm_idx), nodeLoc(2,:,frm_idx),'rs');
    
    for ii = 1:numNode
        text(nodeLoc(1,ii,frm_idx) + 6, nodeLoc(2,ii,frm_idx) - 6, num2str(ii), 'Color', 'r');
    end
    
    for k = 1:nnz(ST)
        plot([nodeLoc(1,idx_i(k),frm_idx),joint_center{idx_i(k),idx_j(k)}(1,frm_idx)],[nodeLoc(2,idx_i(k),frm_idx),joint_center{idx_i(k),idx_j(k)}(2,frm_idx)],'k-');
        plot([joint_center{idx_i(k),idx_j(k)}(1,frm_idx),nodeLoc(1,idx_j(k),frm_idx)],[joint_center{idx_i(k),idx_j(k)}(2,frm_idx),nodeLoc(2,idx_j(k),frm_idx)],'k-');
        plot(joint_center{idx_i(k),idx_j(k)}(1,frm_idx), joint_center{idx_i(k),idx_j(k)}(2,frm_idx),'bo');
        plot(joint_center{idx_i(k),idx_j(k)}(1,frm_idx), joint_center{idx_i(k),idx_j(k)}(2,frm_idx),'bx');
        text(joint_center{idx_i(k),idx_j(k)}(1,frm_idx)-6, joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_j(k)), 'Color', 'b');
        text(joint_center{idx_i(k),idx_j(k)}(1,frm_idx)+6, joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_i(k)), 'Color', 'b');
    end
    axis equal
    pause(0.01);
end

%---------------------------------------------------

cal_motion_dist;

%% Kinematic Structure
KineStruct.width = width;
KineStruct.height = height;
% KineStruct.segGT = segGT;
% KineStruct.seg_idx = seg_idx;
% KineStruct.seg_center = seg_center;
KineStruct.num_seg = numNode;
KineStruct.num_frames = numFrame;
% KineStruct.num_points = num_points;
KineStruct.motion_dist = motion_dist;
KineStruct.affinity = motion_dist / max(max(motion_dist));
KineStruct.structure_i = idx_i;
KineStruct.structure_j = idx_j;
KineStruct.nodeName = node_info;
% KineStruct.videoFileName = videoFileName;
KineStruct.seg_center = nodeLoc;
KineStruct.joint_center = joint_center;

end