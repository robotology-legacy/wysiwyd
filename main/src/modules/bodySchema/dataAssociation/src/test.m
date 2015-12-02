%%
 % Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 % Authors: Martina Zambelli, Hyung Jin Chang
 % email:   m.zambelli13@imperial.ac.uk, hj.chang@imperial.ac.uk
 % Permission is granted to copy, distribute, and/or modify this program
 % under the terms of the GNU General Public License, version 2 or any
 % later version published by the Free Software Foundation.
 %
 % A copy of the license can be found at
 % wysiwyd/license/gpl.txt
 %
 % This program is distributed in the hope that it will be useful, but
 % WITHOUT ANY WARRANTY; without even the implied warranty of
 % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 % Public License for more details
%%





name_idx = ones(539,1)*-1;
name_idx(80:110) = 3;
name_idx(165:200) = 2;
name_idx(250:270) = 1;
name_idx(320:350) = 3;
name_idx(395:410) = 3;
name_idx(450:475) = 1;
name_idx(520:535) = 1;

joint_data = joints(:,1:7);
d_joint_data = joint_data(2:end,:) - joint_data(1:end-1,:);

joint_moving_idx = ones(539,1)*-1;
joint_moving_idx(80:110) = 6;
joint_moving_idx(165:200) = 3;
joint_moving_idx(250:270) = 2;
joint_moving_idx(320:350) = 4;
joint_moving_idx(395:410) = 5;
joint_moving_idx(450:475) = 1;
joint_moving_idx(520:535) = 0;



% %% Hashmap
% 
% keySet = {'hand','forearm','upperarm'};
% 
% valueSet_joints = {[4,5,6],3,[0,1,2]};
% mapObj_joints_name = containers.Map(keySet,valueSet_joints);
% joints_of_body_part = mapObj_joints_name('hand')
% 
% valueSet_touch = {c(find(c==1)),c(find(c==2)),c(find(c==3))};
% mapObj_taxels_part = containers.Map(keySet,valueSet_touch);
% taxels_of_body_part = mapObj_taxels_part('hand')


%%
dataAssocMtx = zeros(7,3,3);
num_sum = 0;
for i=1:538 
    if joint_moving_idx(i)~=-1 && name_idx(i)~=-1 && name_idx(i)~=-1
        dataAssocMtx(joint_moving_idx(i)+1,name_idx(i),name_idx(i)) = dataAssocMtx(joint_moving_idx(i)+1,name_idx(i),name_idx(i)) + 1;
        num_sum = num_sum + 1;
    end
end

dataAssocMtx = dataAssocMtx / num_sum;
[x,y,z] = meshgrid(1:7,1:3,1:3);

figure
% scatter3(x(:),y(:),z(:),400,dataAssocMtx(:),'filled')
hold on
grid on
for x = 1:7
    for y = 1:3
        for z = 1:3
            plot3(x,y,z,'ro','MarkerSize', dataAssocMtx(x,y,z)*100+1, 'LineWidth',2)
            
        end
    end
end

xlabel('joint index')
ylabel('words')
zlabel('touch index')
colormap(winter)

% A = permute(dataAssocMtx,[3,2,1]);
% plot3(A(1,:),A(2,:),A(3,:),'o')
