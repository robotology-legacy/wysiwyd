global kernel_param
global kernel_type
global SET

% %%
% Landgrebe_gen_outlier_start = tic;
% dist_buf = zeros(length(x_SVDD),length(x_SVDD));
% for i=1:length(x_SVDD)
%     for j=1:length(x_SVDD)
%         dist_buf(i,j) = sqrt(sum((x_SVDD(i,:)-x_SVDD(j,:)).^2));
%     end
% end
% dist_buf(dist_buf == 0) = inf;
% mean_min_dist = mean(min(dist_buf));
% 
% %%
% num_uniform_outlier = length(x_SVDD) * 2;
% dist_buf_buf = zeros(length(x_SVDD),1);
% num_ao = 1;
% uniform_outlier = [];
% uniform_outlier_label = [];
% 
% while(1)
%     uniform_outlier_x = rand(1,1) * width;
%     uniform_outlier_y = rand(1,1) * height;
% 
%     for i = 1:length(x_SVDD)
%         dist_buf_buf(i) = sqrt(sum((x_SVDD(i,:)-[uniform_outlier_x,uniform_outlier_y]).^2));
%     end
%     min_dist_buf = min(dist_buf_buf);
% 
% %     if min_dist_buf >= mean_min_dist/10
%     if min_dist_buf < 30
%         uniform_outlier = [uniform_outlier;[uniform_outlier_x,uniform_outlier_y]];
%         uniform_outlier_label = [uniform_outlier_label;-1];
%         num_ao = num_ao + 1;
%     else
% %         disp('less than min dist...');
%     end
% 
%     if num_ao == num_uniform_outlier+1
%         break;
%     end
% end
% Landgrebe_gen_outlier_lapsed = toc(Landgrebe_gen_outlier_start);
% Landgrebe_gen_outlier_time_buf = [Landgrebe_gen_outlier_time_buf, Landgrebe_gen_outlier_lapsed];

%---------------------------------------------------------------------
%%
% Circular gen

if kernel_param == kernel_min
    disp('Gen. Outliers!');
Landgrebe_gen_outlier_start = tic;
center_point = round(mean(x_SVDD));
dist_buf = zeros(length(x_SVDD),1);
for i=1:length(x_SVDD)
    dist_buf(i) = sqrt(sum((x_SVDD(i,:) - center_point).^2));
end
max_dist = max(dist_buf);

%%
num_uniform_outlier = length(x_SVDD) * 5;
dist_buf2 = 0;
num_ao = 1;
uniform_outlier = [];
uniform_outlier_label = [];

while(1)
    uniform_outlier_x = rand(1,1) * width;
    uniform_outlier_y = rand(1,1) * height;
    
    dist_buf2 = sqrt(sum(([uniform_outlier_x,uniform_outlier_y] - center_point).^2));
    
    if dist_buf2 < max_dist * 1.1
        uniform_outlier = [uniform_outlier;[uniform_outlier_x,uniform_outlier_y]];
        uniform_outlier_label = [uniform_outlier_label;-1];
        num_ao = num_ao + 1;
    end
    
    if num_ao == num_uniform_outlier+1
        break;
    end
end
Landgrebe_gen_outlier_lapsed = toc(Landgrebe_gen_outlier_start);
Landgrebe_gen_outlier_time_buf = [Landgrebe_gen_outlier_time_buf, Landgrebe_gen_outlier_lapsed];

%%
% figure(4)
% clf
% plot(x_SVDD(:,1), x_SVDD(:,2),'ro','LineWidth',3, 'MarkerFaceColor','r');
% hold on
% plot(uniform_outlier(:,1), uniform_outlier(:,2),'bx');
% 
% h_legend = legend('feature points','artificial outliers');
% set(h_legend,'FontSize',14);
% xlabel('width');
% ylabel('height');
% axis([0 width 0 height]);
% axis equal;
% 
% save_path = '/home/hjchang/Dropbox/paper_writing/[2015][Journal][TPAMI] Kinematic Structure Learning/paper/figs/fig_kernel_param_select';
% saveas(gcf, [save_path, '/outliers.pdf'], 'pdf')

end

%%
test_start = tic;
test_set = [x_SVDD;uniform_outlier];
test_set_label = [y_SVDD;uniform_outlier_label];
num_test_set = length(test_set);

result = [];
num_TP = 0;
num_FP = 0;
num_FN = 0;
num_TN = 0;

for i=1:num_test_set
    %     disp(kernel_param);
    buf = testSVDD(test_set(i,:),SET,kernel_param,kernel_type);
    
    %     [buf, test_set_label(i)]
%     if buf < 0 && test_set_label(i) >= 0   % TP
    if buf <= -prec && test_set_label(i) == 1   % TP
        num_TP = num_TP + 1;
%     elseif buf < 0 && test_set_label(i) < 0   % FP
    elseif buf <= -prec && test_set_label(i) == -1   % FP
        num_FP = num_FP + 1;
%     elseif buf >= 0 && test_set_label(i) >= 0   % FN
    elseif buf > -prec && test_set_label(i) == 1   % FN
        num_FN = num_FN + 1;
%     elseif buf >= 0 && test_set_label(i) < 0   % TN
    elseif buf > -prec && test_set_label(i) == -1   % TN
        num_TN = num_TN + 1;
    end
end

TPr = num_TP / length(x_SVDD);
FPr = num_FP / length(uniform_outlier);
FNr = num_FN / length(x_SVDD);
TNr = num_TN / length(uniform_outlier);

test_lapsed = toc(test_start);
test_time_buf = [test_time_buf,test_lapsed];