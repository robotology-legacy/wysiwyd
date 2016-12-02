%Generate data for the tests from the ground truth
% - ngroup      number of groups
% - N           points in each group
% - xord        normalized coordinates ordered per group
% - yord        unnormalized coordinates ordered per group
% - xordindex   vector of indeces to pass from unordered coordinates to
%               coordinates ordered per group
% - x1ord       coordinates normalized with Hartley's normalization
%

%order the true segmentation in groups
% % % % % ngroups=max(s);                                             %total number of groups
% % % % % xordindex=[];
% % % % % N=[];
% % % % % for (i=1:ngroups)
% % % % %     xordindex=[xordindex; find(s==i)];                      %index which pass from non-ordered to ordered features
% % % % %     N=[N length(find(s==i))];                               %array with the number of features for each group
% % % % % end

ngroups=max(s);                                             %total number of groups
xordindex=[];
N=[];
for (i=1:ngroups)
    xordindex=[xordindex; find(s==i)];                      %index which pass from non-ordered to ordered features
    N=[N length(find(s==i))];                               %array with the number of features for each group
end

% x = x(:,:,1:4);
% y = y(:,:,1:4);

xord=x(:,xordindex,:);
yord=y(:,xordindex,:);

clear x1ord;
% 
% for(i=1:size(x,3))
%     x1ord(:,:,i)=normalise2dpts(yord(:,:,i));
% end
