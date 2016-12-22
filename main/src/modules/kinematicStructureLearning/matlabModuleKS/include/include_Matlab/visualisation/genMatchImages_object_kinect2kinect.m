function img_output = genMatchImages_object_kinect2kinect(cdata_P,cdata_Q,KineStruct_P, KineStruct_Q, data_source_P, data_source_Q, X, method)

height_normalised = 240;
width_normalised = 320;
% height_normalised = 480;
% width_normalised = 640;

%%
% draw image
h_result = figure(1004);
iptsetpref('ImshowBorder','tight')
% imshow(img_combined,'Border','tight');

% Drawing the connections with color segments
color_idx = 'rgbcmy';
% marker_idx = '+o*xsd^v><ph';
marker_idx = '............';

color_value = 0.99;

frm_idx_P = KineStruct_P.num_frames;
frm_idx_Q = KineStruct_Q.num_frames;

numNode = 13;

connection = zeros(numNode, numNode);
connection(1,2) = 1;
connection(2,3) = 1;
connection(3,4) = 1;
connection(4,5) = 1;
connection(2,6) = 1;
connection(6,7) = 1;
connection(7,8) = 1;
connection(2,9) = 1;
connection(9,10) = 1;
connection(10,11) = 1;
connection(9,12) = 1;
connection(12,13) = 1;

ST = sparse(connection);
[idx_i, idx_j, value] = find(ST);

for frm_idx = 1:min(frm_idx_P, frm_idx_Q)
    clf
    hold on
    grid on    
    %%
    % drawing connections for Graph P
    for m = 1:size(KineStruct_P.structure_i,1)

        plot3(KineStruct_P.seg_center(1,:,frm_idx), KineStruct_P.seg_center(3,:,frm_idx), KineStruct_P.seg_center(2,:,frm_idx),'rs');
        
        %     for ii = 1:numNode
        %         text(KineStruct_P.seg_center(1,ii,frm_idx) + 6, KineStruct_P.seg_center(2,ii,frm_idx) - 6, num2str(ii), 'Color', 'r');
        %     end
        
        for k = 1:nnz(ST)
            plot3([KineStruct_P.seg_center(1,idx_i(k),frm_idx),KineStruct_P.joint_center{idx_i(k),idx_j(k)}(1,frm_idx)],...
                [KineStruct_P.seg_center(3,idx_i(k),frm_idx),KineStruct_P.joint_center{idx_i(k),idx_j(k)}(3,frm_idx)],...
                [KineStruct_P.seg_center(2,idx_i(k),frm_idx),KineStruct_P.joint_center{idx_i(k),idx_j(k)}(2,frm_idx)],'k-');
            plot3([KineStruct_P.joint_center{idx_i(k),idx_j(k)}(1,frm_idx),KineStruct_P.seg_center(1,idx_j(k),frm_idx)],...
                [KineStruct_P.joint_center{idx_i(k),idx_j(k)}(3,frm_idx),KineStruct_P.seg_center(3,idx_j(k),frm_idx)],...
                [KineStruct_P.joint_center{idx_i(k),idx_j(k)}(2,frm_idx),KineStruct_P.seg_center(2,idx_j(k),frm_idx)],'k-');
            plot3(KineStruct_P.joint_center{idx_i(k),idx_j(k)}(1,frm_idx),...
                KineStruct_P.joint_center{idx_i(k),idx_j(k)}(3,frm_idx),...
                KineStruct_P.joint_center{idx_i(k),idx_j(k)}(2,frm_idx),'bo');
            plot3(KineStruct_P.joint_center{idx_i(k),idx_j(k)}(1,frm_idx),...
                KineStruct_P.joint_center{idx_i(k),idx_j(k)}(3,frm_idx),...
                KineStruct_P.joint_center{idx_i(k),idx_j(k)}(2,frm_idx),'bx');
            %         text(KineStruct_P.joint_center{idx_i(k),idx_j(k)}(1,frm_idx)-6, KineStruct_P.joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_j(k)), 'Color', 'b');
            %         text(KineStruct_P.joint_center{idx_i(k),idx_j(k)}(1,frm_idx)+6, KineStruct_P.joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_i(k)), 'Color', 'b');
        end
        plot3(KineStruct_P.object(1), KineStruct_P.object(3), KineStruct_P.object(2),'gp',...
            'MarkerSize',10,...
            'MarkerEdgeColor','g',...
            'MarkerFaceColor',[0.5,0.5,0.5])
        text(KineStruct_P.object(1), KineStruct_P.object(3), KineStruct_P.object(2)+6,'P', 'Color', 'm')
        
%         axis equal
        %     view(3)
    end
    % drawing connections for Graph Q
    for m = 1:size(KineStruct_Q.structure_i,1)
%         hold on
%         grid on
        plot3(KineStruct_Q.seg_center(1,:,frm_idx), -1*KineStruct_Q.seg_center(3,:,frm_idx), KineStruct_Q.seg_center(2,:,frm_idx),'rs');
        
        %     for ii = 1:numNode
        %         text(KineStruct_Q.seg_center(1,ii,frm_idx) + 6, KineStruct_Q.seg_center(2,ii,frm_idx) - 6, num2str(ii), 'Color', 'r');
        %     end
        
        for k = 1:nnz(ST)
            plot3([KineStruct_Q.seg_center(1,idx_i(k),frm_idx),KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(1,frm_idx)],...
                [-1*KineStruct_Q.seg_center(3,idx_i(k),frm_idx),-1*KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(3,frm_idx)],...
                [KineStruct_Q.seg_center(2,idx_i(k),frm_idx),KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(2,frm_idx)],'k-');
            plot3([KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(1,frm_idx),KineStruct_Q.seg_center(1,idx_j(k),frm_idx)],...
                [-1*KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(3,frm_idx),-1*KineStruct_Q.seg_center(3,idx_j(k),frm_idx)],...
                [KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(2,frm_idx),KineStruct_Q.seg_center(2,idx_j(k),frm_idx)],'k-');
            plot3(KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(1,frm_idx),...
                -1*KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(3,frm_idx),...
                KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(2,frm_idx),'bo');
            plot3(KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(1,frm_idx),...
                -1*KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(3,frm_idx),...
                KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(2,frm_idx),'bx');
            %         text(KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(1,frm_idx)-6, KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_j(k)), 'Color', 'b');
            %         text(KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(1,frm_idx)+6, KineStruct_Q.joint_center{idx_i(k),idx_j(k)}(2,frm_idx)-6,num2str(idx_i(k)), 'Color', 'b');
        end
        plot3(KineStruct_Q.object(1), -1*KineStruct_Q.object(3), KineStruct_Q.object(2),'gp',...
            'MarkerSize',10,...
            'MarkerEdgeColor','g',...
            'MarkerFaceColor',[0.5,0.5,0.5])
        text(KineStruct_Q.object(1), -1*KineStruct_Q.object(3), KineStruct_Q.object(2)+6,'Q', 'Color', 'c')
        
%         axis equal
        %     view(3)
    end
    
    %% Draw matches
    color_value = 0;
    for p = 1:size(X,1)
        for q = 1:size(X,2)
            if X(p,q) > 0
%                 hold on
                plot3([KineStruct_P.seg_center(1,p,frm_idx), KineStruct_Q.seg_center(1,q,frm_idx)],...
                    [KineStruct_P.seg_center(3,p,frm_idx), -1*KineStruct_Q.seg_center(3,q,frm_idx)],...
                    [KineStruct_P.seg_center(2,p,frm_idx), KineStruct_Q.seg_center(2,q,frm_idx)],'-','Color',[color_value,color_value,color_value],...
                    'LineWidth',2,...
                    'MarkerSize',5,...
                    'MarkerEdgeColor',[color_value,color_value,color_value],...
                    'MarkerFaceColor',[1.0,0.2,0.2]);
                plot3([KineStruct_P.seg_center(1,p,frm_idx), KineStruct_Q.seg_center(1,q,frm_idx)],...
                    [KineStruct_P.seg_center(3,p,frm_idx), -1*KineStruct_Q.seg_center(3,q,frm_idx)],...
                    [KineStruct_P.seg_center(2,p,frm_idx), KineStruct_Q.seg_center(2,q,frm_idx)],'-y.',...
                    'LineWidth',1,...
                    'MarkerSize',5,...
                    'MarkerEdgeColor','y',...
                    'MarkerFaceColor',[1.0,0.2,0.2]);
            end
        end
    end
    axis([0, 320, -200, 200, 0, 240]);
    view(140,5)
    pause(0.01)
end
%% Save result
% iptsetpref('ImshowBorder','tight');
% [SUCCESS,MESSAGE,MESSAGEID] = rmdir('result','s');
% mkdir('result/images/correspondences');
% result_save_folder_images_correspondences = 'result/images/correspondences/';
% fig_save_name = [result_save_folder_images_correspondences,'output'];
% % fig_save_name = ['result/realSeq/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'/',KineStruct_P.videoFileName(1:end-4),'-',KineStruct_Q.videoFileName(1:end-4),'-',method];
%
% export_fig(fig_save_name,'-png');
% % F_points = getframe(h_result);
% % [fig_save_name,'.png']
% % imwrite(F_points.cdata,[fig_save_name,'.png']);
%
% img_before = imread([fig_save_name,'.png']);
% img_after = imresize(img_before, [height_normalised, width_normalised*2]);
% imwrite(img_after, [fig_save_name,'.png']);
% %%
% img_output = getimage;

end