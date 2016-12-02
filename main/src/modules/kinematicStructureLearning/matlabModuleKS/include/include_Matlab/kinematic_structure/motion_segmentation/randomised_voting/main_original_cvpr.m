%% start
close all; clear; clc;
addpath 'f_estimation'
addpath 'fast_Kmeans'
addpath 'spectral_clutering'

%% path ????
path = 'Hopkins155';
sequence = {'1R2RC', '1R2RC_g12', '1R2RC_g13', '1R2RC_g23', '1R2RCR', '1R2RCR_g12', '1R2RCR_g13', '1R2RCR_g23', '1R2RCT_A', ...
    '1R2RCT_A_g12', '1R2RCT_A_g13', '1R2RCT_A_g23', '1R2RCT_B', '1R2RCT_B_g12', '1R2RCT_B_g13', '1R2RCT_B_g23', '1R2TCR', ...
    '1R2TCR_g12', '1R2TCR_g13', '1R2TCR_g23', '1R2TCRT', '1R2TCRT_g12', '1R2TCRT_g13', '1R2TCRT_g23', '1RT2RCR', '1RT2RCR_g12', ...
    '1RT2RCR_g13', '1RT2RCR_g23', '1RT2RCRT', '1RT2RCRT_g12', '1RT2RCRT_g13', '1RT2RCRT_g23', '1RT2RTCRT_A', '1RT2RTCRT_A_g12', ...
    '1RT2RTCRT_A_g13', '1RT2RTCRT_A_g23', '1RT2RTCRT_B', '1RT2RTCRT_B_g12', '1RT2RTCRT_B_g13', '1RT2RTCRT_B_g23', '1RT2TC', ...
    '1RT2TC_g12', '1RT2TC_g13', '1RT2TC_g23', '1RT2TCRT_A', '1RT2TCRT_A_g12', '1RT2TCRT_A_g13', '1RT2TCRT_A_g23', '1RT2TCRT_B', ...
    '1RT2TCRT_B_g12', '1RT2TCRT_B_g13', '1RT2TCRT_B_g23', '2R3RTC', '2R3RTC_g12', '2R3RTC_g13', '2R3RTC_g23', '2R3RTCRT', ...
    '2R3RTCRT_g12', '2R3RTCRT_g13', '2R3RTCRT_g23', '2RT3RC', '2RT3RC_g12', '2RT3RC_g13', '2RT3RC_g23', '2RT3RCR', '2RT3RCR_g12', ...
    '2RT3RCR_g13', '2RT3RCR_g23', '2RT3RCT_A', '2RT3RCT_A_g12', '2RT3RCT_A_g13', '2RT3RCT_A_g23', '2RT3RCT_B', '2RT3RCT_B_g12', ...
    '2RT3RCT_B_g13', '2RT3RCT_B_g23', '2RT3RTCRT', '2RT3RTCRT_g12', '2RT3RTCRT_g13', '2RT3RTCRT_g23', '2T3RCR', '2T3RCR_g12', ...
    '2T3RCR_g13', '2T3RCR_g23', '2T3RCRT', '2T3RCRT_g12', '2T3RCRT_g13', '2T3RCRT_g23', '2T3RCRTP', '2T3RCRTP_g12', '2T3RCRTP_g13', ...
    '2T3RCRTP_g23', '2T3RCTP', '2T3RCTP_g12', '2T3RCTP_g13', '2T3RCTP_g23', '2T3RTCR', '2T3RTCR_g12', '2T3RTCR_g13', '2T3RTCR_g23', ...
    'arm', 'articulated', 'articulated_g12', 'articulated_g13', 'articulated_g23', 'cars1', 'cars2', 'cars2_06', 'cars2_06_g12', ...
    'cars2_06_g13', 'cars2_06_g23', 'cars2_07', 'cars2_07_g12', 'cars2_07_g13', 'cars2_07_g23', 'cars2B', 'cars2B_g12', ...
    'cars2B_g13', 'cars2B_g23', 'cars3', 'cars3_g12', 'cars3_g13', 'cars3_g23', 'cars4', 'cars5', 'cars5_g12', 'cars5_g13', ...
    'cars5_g23', 'cars6', 'cars7', 'cars8', 'cars9', 'cars9_g12', 'cars9_g13', 'cars9_g23', 'cars10', 'cars10_g12', 'cars10_g13', ...
    'cars10_g23', 'head', 'kanatani1', 'kanatani2', 'kanatani3', 'people1', 'people2', 'three-cars', 'three-cars_g12', ...
    'three-cars_g13', 'three-cars_g23', 'truck1', 'truck2', 'two_cranes', 'two_cranes_g12', 'two_cranes_g13', 'two_cranes_g23' ...
    'oc1R2RC', 'oc1R2RC_g12', 'oc1R2RC_g13', 'oc1R2RC_g23', 'oc1R2RCT', 'oc1R2RCT_g12', 'oc1R2RCT_g13', 'oc1R2RCT_g23', 'oc2R3RCRT', ...
    'oc2R3RCRT_g12', 'oc2R3RCRT_g13', 'oc2R3RCRT_g23', ...
    'nrbooks3', 'carsbus', 'books', 'carsTurning'};

for zz=1:1 % the number of experiments
    seqseq = 1;
    startt = tic;
    
    for seq=152 % the number of sequence
        %% loading
        sequencename = sequence{seq};
        fprintf('*********** Experiment #%d, Sequence #%d, %s ***********\n', zz, seq, sequencename);
        fullpath=fullfile(path,sequencename);

        if(~exist(fullpath,'dir'))
            error(['Project directory ''' sequencename '''doesn''t exist']);
        end

        fullfilename = sprintf('%s/%s_truth.mat', fullpath, sequencename);
        load(fullfilename);

        %% generate test data
        generate_test_data;

        %% parameter setting
        c = max(s);
%         c = 4;
        T_i = 200; % the number of iteration     % 150
        T = 10; % the number of trial
        T_c = 20; % convergence test, noise free : 15, noise : 5
        T_r = 15; % reinitialization at T_r
        alpha = 0.9; % decay parameter
        lambda = 2; % voting strength

        d = 1;
        converge_check2 = 0;

        max_dist = ones(1,c);
        best_overlap = 0;

        % generate noise, 0 means "no noise"
        y = generate_noise(y,0);

        % y is trajectory, 3 x (the number of points) x (the number of frames)
        % T, T_i, T_c, T_r, alpha, lambda : tuning parameters
        % c : the number of group
        % s : ground truth, (the number of points) x 1
        
%         final = motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, c, s);
        final = my_motion_segmentation(y, T, T_i, T_c, T_r, alpha, lambda, c);
        
        %%
        color_idx = 'rgbcmykrbgcmyk';
        figure(1000)
        for f=1:size(y,3)
            for p=1:size(y,2)
                plot(y(1,p,f), y(2,p,f),'.','Color',color_idx(final(p)));
                axis([0, width, 0, height]);
                hold on
            end
            pause(0.03);
            clf
        end        

        fprintf('\n- excution time (sec) :\n');
        ex_time(zz) = toc(startt)
        cur_error_rate = compare_labels(s,final);

        fprintf('\n- error rate per sequence (%%) :\n');
        error_rate(seqseq,zz) = cur_error_rate;
        error_rate' .* 100

    %     iter
        fprintf('\n- total error rate : %.3f %%\t', mean(error_rate).*100);
        fprintf('\n\n');
        group_number(seqseq) = max(s);

        seqseq = seqseq + 1;
    end
%save '131027'
end