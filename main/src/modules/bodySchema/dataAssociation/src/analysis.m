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






%% Analysis

addpath(genpath('../include/AffinityPropagation'))

%%

joints = joints(2:end,:) - joints(1:end-1,:);
touch = touch(1:end-1,:);
% touch = touch/255;
touch = mat2gray(randn(size(touch)));

T = touch'*touch;


%% Clustering on T

data = T;

N = size(data,1);
s = zeros(N,N);
for i=1:N
    for k=1:N
        s(i,k) = -norm((data(i,:)-data(k,:)),2);
    end
end

init_pref = min(s(s~=0));    % set the preference value as a common value of median of s
init_pref = init_pref * log(N);

[idx,netsim,dpsim,expref] = apcluster(s,init_pref);
% [idx,netsim,dpsim,expref] = apclusterSparse(s,init_pref);

I = unique(idx);
c = idx;
K = length(I);
for i=1:K
    c(idx==I(i)) = i;
end

color_idx = 'yrgbmc';
marker_idx = 'ox+*dv^<>ph';

figure(100)
clf
for i=1:K
    plot(data(c==i,1),data(c==i,2),'Color',color_idx(mod(i,6)+1),'Marker',marker_idx(mod(i,11)+1));
    hold on
    plot(data(I(i),1),data(I(i),2),'Color','k','Marker','s', 'MarkerSize',10);
end


%%
N = size(touch,1);
touch_thres = 0.7;
firing_taxel = touch > touch_thres;
firing_taxel_label = firing_taxel .* repmat(c',[N,1]);
% firing_taxel_label_sort = sort(firing_taxel_label,2,'descend');

numLabel = zeros(N,length(unique(c)));
for i=1:length(unique(c))
    for n = 1:N    
        numLabel(n,i) = length(find(firing_taxel_label(n,:) == i));
    end
end

[I,labels] = max(numLabel,[],2);
