
trial = num_frames;

dist_mtx_fundmtx = zeros(num_seg,num_seg);

% for trial_counter = 1:trial
%     rf = randperm(num_frames,2);

for frm = 1:num_frames-1
    rf = [frm, frm+1];
    rand_frm_1 = rf(1);
    rand_frm_2 = rf(2);

    for i=1:num_seg

        x_i_rf1 = y(:,seg_idx{i},rand_frm_1);
        x_i_rf2 = y(:,seg_idx{i},rand_frm_2);

        % estimate F_g using selected m points
        [F_g,e1,e2] = fundmatrix(x_i_rf1,x_i_rf2);

        for j=i+1:num_seg
            x_j_rf1 = y(:,seg_idx{j}, rand_frm_1);
            x_j_rf2 = y(:,seg_idx{j}, rand_frm_2);

            dist = SampSonDist(F_g, x_j_rf1, x_j_rf2);
            dist = exp(-dist);
%             dist_mtx_fundmtx(i,j) = dist_mtx_fundmtx(i,j) + mean(dist);
            dist_mtx_fundmtx(i,j) = dist_mtx_fundmtx(i,j)...
                                    + mean(dist)...
                                    *((norm([seg_center(1:2,i,rand_frm_1)-seg_center(1:2,j,rand_frm_1)],2))...
                                    +(norm([seg_center(1:2,i,rand_frm_2)-seg_center(1:2,j,rand_frm_2)],2)));
        end
    end
end

%%
idx1 = [];
idx2 = [];
W_buf = [];
k_knn = num_seg-1;
% k_knn = 30;
for i = 1:seg_y_num
    [Y,I] = sort(dist_final(i,:));
    for l = 1:k_knn
        idx1 = [idx1,i];
        idx2 = [idx2,I(l+1)];
        W_buf = [W_buf,dist_mtx_fundmtx(i,I(l+1))];
    end
end

DG = sparse(idx1,idx2,W_buf);
UG = tril(DG + DG');

% Find the minimum spanning tree of UG
[ST,pred] = graphminspantree(UG);
[ii,jj,ss] = find(ST);

% Drawing the connections
for frm = 1:seg_y_frm;
    figure(222)
    clf
    plot(y(1,:,frm),height-y(2,:,frm),'b.');
    %     plot(y(1,:,frm),y(2,:,frm),'b.');
    hold on
    plot(seg_center(1,:,frm),height-seg_center(2,:,frm),'ro');
    %     plot(seg_y(1,:,frm),seg_y(2,:,frm),'ro');
    for i = 1:seg_y_num
        text(seg_center(1,i,frm)+3, height-seg_center(2,i,frm)+3, num2str(i), 'Color', 'r');
    end
    axis([0 width 0 height]);    
    hold on
    for m = 1:size(ii,1)
        plot([seg_center(1,ii(m),frm),seg_center(1,jj(m),frm)],height-[seg_center(2,ii(m),frm),seg_center(2,jj(m),frm)],'k-');
        %         plot([seg_y(1,ii(m),frm),seg_y(1,jj(m),frm)],[seg_y(2,ii(m),frm),seg_y(2,jj(m),frm)],'k-');
    end
    pause(0.001)
end