nAlg = 0; 
%% HGM - 1
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @HGM;
    Alg(nAlg).strName = 'HGM';
    Alg(nAlg).bOrder = [0 0 1];
    % Display option
    Alg(nAlg).color = 'c'; % color
    Alg(nAlg).lineStyle = '-.'; % linestyle
    Alg(nAlg).marker = 'x'; % linestyle
end
%% Tensor Matching - 2
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @tensorMatching;
    Alg(nAlg).strName = 'TM';
    Alg(nAlg).bOrder = [0 0 1];
    Alg(nAlg).stoc = 'doubly'; % single
    % Display option
    Alg(nAlg).color = 'g'; % color
    Alg(nAlg).lineStyle = ':'; % linestyle
    Alg(nAlg).marker = 'x'; % linestyle
end
%% RRWHM - 3
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @RRWHM;
    Alg(nAlg).strName = 'RRWHM';
%     Alg(nAlg).bOrder = [0 0 1];
%     Alg(nAlg).bOrder = [0 1 0];
%     Alg(nAlg).bOrder = [1 0 0];
    Alg(nAlg).bOrder = [1 1 1];
%     Alg(nAlg).bOrder = [0 1 1]; 
    % Display option
    Alg(nAlg).color = 'b'; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 's'; % linestyle
end
%% BCAGM - 4
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).strName = 'BCAGM';
%     Alg(nAlg).bOrder = [0 0 1];
    Alg(nAlg).bOrder = [1 1 1];
%     Alg(nAlg).bOrder = [0 1 1];
    % Display option
    Alg(nAlg).color = 'r'; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 'o'; % linestyle
end
%% BCAGM+MP - 5
if 1   
    nAlg = nAlg + 1;
    Alg(nAlg).strName = 'BCAGM+MP';
%     Alg(nAlg).bOrder = [0 0 1];
    Alg(nAlg).bOrder = [1 1 1];
%     Alg(nAlg).bOrder = [0 1 1];
    % Display option
    Alg(nAlg).color = 'm'; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 'o'; % linestyle
end
%% BCAGM+IPFP - 6
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).strName = 'BCAGM+IPFP';
%     Alg(nAlg).bOrder = [0 0 1];
    Alg(nAlg).bOrder = [1 1 1];
%     Alg(nAlg).bOrder = [0 1 1];
    % Display option
    Alg(nAlg).color = [0.5 0 1]; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 'o'; % linestyle
end
%% MPM - 7
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @MPM;
    Alg(nAlg).variable = {'affinityMatrix', 'group1', 'group2'};
    Alg(nAlg).param = {};
    Alg(nAlg).strName = 'MPM';
    % Display option
    Alg(nAlg).color = 'y'; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = '*'; % linestyle
end
%% RRWM - 8
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @RRWM;
    Alg(nAlg).variable = {'affinityMatrix', 'group1', 'group2'};
    Alg(nAlg).param = {};
    Alg(nAlg).strName = 'RRWM';
    % Display option
    Alg(nAlg).color = [1 0.5 0.5]; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 's'; % linestyle
end
%% IPFP - 9
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @ipfp_gm;
    Alg(nAlg).variable = {'affinityMatrix', 'L12'};
    Alg(nAlg).param = {};
    Alg(nAlg).strName = 'IPFP';
    % Display option
    Alg(nAlg).color = [0.5 0.6 0.7]; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 'x'; % linestyle
end
%% SM - 10
if 1
    nAlg = nAlg + 1;
    Alg(nAlg).fhandle = @SM;
    Alg(nAlg).variable = {'affinityMatrix'};
    Alg(nAlg).param = {};
    Alg(nAlg).strName = 'SM';
    % Display option
    Alg(nAlg).color = [0 0.5 0.9]; % color
    Alg(nAlg).lineStyle = '-'; % linestyle
    Alg(nAlg).marker = 'x'; % linestyle
end
%%
clear nAlg
