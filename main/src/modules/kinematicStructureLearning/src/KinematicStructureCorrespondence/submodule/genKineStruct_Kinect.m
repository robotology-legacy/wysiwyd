function KineStruct = genKineStruct_Kinect(idx)
%%
% Load file from the filepath
disp('=========================================================================');
disp(['Perform Kinematic Structure Building from Kinect data...',idx]);
disp('=========================================================================');

edge_file = 'submodule/KinectMapping/edges.txt';
joint_file = ['ABM/',idx,'/joints.txt'];
node_file = 'submodule/KinectMapping/nodes.txt';

edge_info = importdata(edge_file);
joint_info = importdata(joint_file);
node_info = importdata(node_file);

num_frames = length(unique(joint_info(:,1)));
num_seg = length(unique(joint_info(:,2)));
num_edges = length(edge_info);

height = 240;
width = 320;

joints = cell(num_frames,1);

for frm_idx = 1:num_frames
    joints{frm_idx} = [(joint_info((frm_idx-1)*num_seg+1:frm_idx*num_seg,3)+1) * width/2,...
        (joint_info((frm_idx-1)*num_seg+1:frm_idx*num_seg,4)+1) * height/2,...
        (joint_info((frm_idx-1)*num_seg+1:frm_idx*num_seg,5)) * 100];
end

%% Draw
figure(739)
clf
for frm_idx = 1:num_frames
    clf
    hold on
    grid on
    for node_idx = 1:num_seg
        plot3(joints{frm_idx}(node_idx,1), joints{frm_idx}(node_idx,2), joints{frm_idx}(node_idx,3), 'ro');
        %         text(joints{frm_idx}(node_idx,1), joints{frm_idx}(node_idx,2), node_info{node_idx}, 'Color', 'b');
    end
    
    for edge_idx = 1:num_edges
        plot3([joints{frm_idx}(edge_info(edge_idx,1),1),joints{frm_idx}(edge_info(edge_idx,2),1)],...
            [joints{frm_idx}(edge_info(edge_idx,1),2),joints{frm_idx}(edge_info(edge_idx,2),2)],...
            [joints{frm_idx}(edge_info(edge_idx,1),3),joints{frm_idx}(edge_info(edge_idx,2),3)],'k-');
    end
    
    axis equal
    view(3)
    pause(0.01);
end

%% New Nodes & Joints
numNode = 11;
numFrame = num_frames;

nodeLoc = zeros(3, numNode, numFrame);

joint_center_Kinect = cell(numNode, numNode);

for i = 1:num_seg
    for j = 1:num_seg
        if i~=j
            buf = zeros(3,numFrame);
            for frm_idx = 1:numFrame
                buf(1,frm_idx) = (joints{frm_idx}(i,1) + joints{frm_idx}(j,1)) / 2;
                buf(2,frm_idx) = (joints{frm_idx}(i,2) + joints{frm_idx}(j,2)) / 2;
                buf(3,frm_idx) = (joints{frm_idx}(i,3) + joints{frm_idx}(j,3)) / 2;
            end
            joint_center_Kinect{i,j} = buf;
        end
    end
end

for frm_idx = 1 : numFrame
    nodeLoc(:,1,frm_idx) = joints{frm_idx}(9,:)';
    nodeLoc(:,2,frm_idx) = joints{frm_idx}(4,:)';
    nodeLoc(:,7,frm_idx) = joints{frm_idx}(1,:)';
    nodeLoc(:,4,frm_idx) = joints{frm_idx}(10,:)';
    nodeLoc(:,6,frm_idx) = joints{frm_idx}(11,:)';
    
    nodeLoc(:,3,frm_idx) = joint_center_Kinect{14,2}(:,frm_idx)';
    nodeLoc(:,5,frm_idx) = joint_center_Kinect{15,3}(:,frm_idx)';
    nodeLoc(:,8,frm_idx) = joint_center_Kinect{7,5}(:,frm_idx)';
    nodeLoc(:,9,frm_idx) = joint_center_Kinect{12,5}(:,frm_idx)';
    nodeLoc(:,10,frm_idx) = joint_center_Kinect{8,6}(:,frm_idx)';
    nodeLoc(:,11,frm_idx) = joint_center_Kinect{13,6}(:,frm_idx)';
    
%     nodeLoc(:,12,frm_idx) = joints{frm_idx}(16,:)';  % CoM
end

joint_center = cell(numNode, numNode);

for i = 1:numNode
    for j = 1:numNode
        if i~=j
            buf = zeros(3,numFrame);
            for frm_idx = 1:numFrame
                buf(1,frm_idx) = (nodeLoc(1,i,frm_idx) + nodeLoc(1,j,frm_idx)) / 2;
                buf(2,frm_idx) = (nodeLoc(2,i,frm_idx) + nodeLoc(2,j,frm_idx)) / 2;
                buf(3,frm_idx) = (nodeLoc(3,i,frm_idx) + nodeLoc(3,j,frm_idx)) / 2;
            end
            joint_center{i,j} = buf;
        end
    end
end

%% Joint assignment by hand
idx_list = [2,  3,  2  ;...
    3,  2,  2  ;...
    3,  4,  14   ;...
    4,  3,  14   ;...
    2,  5,  3   ;...
    5,  2,  3   ;...
    5,  6,  15   ;...
    6,  5,  15   ;...
    7,  8,  7  ;...
    8,  7,  7  ;...
    8,  9,  5  ;...
    9,  8,  5  ;...
    7,  10, 8   ;...
    10, 7,  8   ;...
    10, 11, 6   ;...
    11, 10, 6   ];

for idx = 1:16
    i = idx_list(idx,1);
    j = idx_list(idx,2);
    kinect_jnt_idx = idx_list(idx,3);
    buf = zeros(3,numFrame);
    
    for frm_idx = 1:numFrame
        buf(1,frm_idx) = joints{frm_idx}(kinect_jnt_idx,1);
        buf(2,frm_idx) = joints{frm_idx}(kinect_jnt_idx,2);
        buf(3,frm_idx) = joints{frm_idx}(kinect_jnt_idx,3);
    end
    joint_center{i,j} = buf;
end

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
%%
%---------------------------------------------------
figure(705)

for frm_idx = 1:numFrame
    clf
    hold on
    grid on
    plot3(nodeLoc(1,:,frm_idx), nodeLoc(2,:,frm_idx), nodeLoc(3,:,frm_idx),'rs');
    
    %     for ii = 1:numNode
    %         text(nodeLoc(1,ii,frm_idx) + 6, nodeLoc(2,ii,frm_idx) - 6, num2str(ii), 'Color', 'r');
    %     end
    
    for k = 1:nnz(ST)
        plot3([nodeLoc(1,idx_i(k),frm_idx),joint_center{idx_i(k),idx_j(k)}(1,frm_idx)],...
            [nodeLoc(2,idx_i(k),frm_idx),joint_center{idx_i(k),idx_j(k)}(2,frm_idx)],...
            [nodeLoc(3,idx_i(k),frm_idx),joint_center{idx_i(k),idx_j(k)}(3,frm_idx)],'k-');
        plot3([joint_center{idx_i(k),idx_j(k)}(1,frm_idx),nodeLoc(1,idx_j(k),frm_idx)],...
            [joint_center{idx_i(k),idx_j(k)}(2,frm_idx),nodeLoc(2,idx_j(k),frm_idx)],...
            [joint_center{idx_i(k),idx_j(k)}(3,frm_idx),nodeLoc(3,idx_j(k),frm_idx)],'k-');
        plot3(joint_center{idx_i(k),idx_j(k)}(1,frm_idx),...
            joint_center{idx_i(k),idx_j(k)}(2,frm_idx),...
            joint_center{idx_i(k),idx_j(k)}(3,frm_idx),'bo');
        plot3(joint_center{idx_i(k),idx_j(k)}(1,frm_idx),...
            joint_center{idx_i(k),idx_j(k)}(2,frm_idx),...
            joint_center{idx_i(k),idx_j(k)}(3,frm_idx),'bx');
        %         text(joint_center{idx_i(k),idx_j(k)}(1,frm_idx)-6, joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_j(k)), 'Color', 'b');
        %         text(joint_center{idx_i(k),idx_j(k)}(1,frm_idx)+6, joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_i(k)), 'Color', 'b');
    end
    axis equal
    view(3)
    pause(0.01);
end

%---------------------------------------------------

cal_motion_dist;

%% Kinematic Structure
KineStruct.width = width;
KineStruct.height = height;
KineStruct.num_seg = numNode;
KineStruct.num_frames = numFrame;
KineStruct.motion_dist = motion_dist;
KineStruct.affinity = motion_dist / max(max(motion_dist));
KineStruct.structure_i = idx_i;
KineStruct.structure_j = idx_j;
KineStruct.nodeName = node_info;
KineStruct.seg_center = nodeLoc;
KineStruct.joint_center = joint_center;

% end