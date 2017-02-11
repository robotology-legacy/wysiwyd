%%
% Start location
kernel_param_start = width / 10;
kernel_param_step = width / 50;
kernel_param_max = width / 2;
kernel_param_min = width / 50;

%% Simulated Annealing
% Number of cycles
n = 50;
% Number of trials per cycle
m = 50;
% Number of accepted solutions
na = 0.0;
% Probability of accepting worse solution at the start
p_start = 0.7;
% Probability of accepting worse solution at the end
p_end = 0.001;
% Initial temperature
t_start = -1.0/log(p_start);
% Final temperature
t_end = -1.0/log(p_end);
% Fractional reduction every cycle
frac = (t_end/t_start)^(1.0/(n-1.0));
% Initialize x
kernel_param_values = zeros(n+1,1);
kernel_param_values(1,:) = kernel_param_start;
kernel_param_i = kernel_param_start;
na = na + 1.0;
% Current best results so far
kernel_param_c = kernel_param_values(1,:);
fc = f(x_SVDD,y_SVDD,C,kernel_type,kernel_param_i, SET);
fs = zeros(n+1,1);
fs(1,:) = fc;
% Current temperature
t = t_start;
% DeltaE Average
DeltaE_avg = 0.0;
for i=1:n
    disp(['Cycle: ',num2str(i),' with Temperature: ',num2str(t)])
    for j=1:m
        % Generate new trial points
        kernel_param_i(1) = kernel_param_c(1) + kernel_param_step * (rand() - 0.5);
        % Clip to upper and lower bounds
        kernel_param_i(1) = max(min(kernel_param_i(1),kernel_param_max),kernel_param_min);
        DeltaE = abs(f(x_SVDD,y_SVDD,C,kernel_type,kernel_param_i, SET)-fc);
        if (f(x_SVDD,y_SVDD,C,kernel_type,kernel_param_i, SET)>fc)
            %             % Initialize DeltaE_avg if a worse solution was found
            %             %   on the first iteration
            if (i==1 && j==1)
                DeltaE_avg = DeltaE;
            end
            % objective function is worse
            % generate probability of acceptance
            p = exp(-DeltaE/(DeltaE_avg * t));
            %             % determine whether to accept worse point
            if (rand()<p)
                % accept the worse solution
                accept = true;
            else
                % don't accept the worse solution
                accept = false;
            end
        else
            % objective function is lower, automatically accept
            accept = true;
        end
        if (accept==true)
            % update currently accepted solution
            kernel_param_c(1) = kernel_param_i(1);
            fc = f(x_SVDD,y_SVDD,C,kernel_type,kernel_param_c, SET);
            % increment number of accepted solutions
            na = na + 1.0;
            % update DeltaE_avg
            DeltaE_avg = (DeltaE_avg * (na-1.0) +  DeltaE) / na;
        end
    end
    % Record the best x values at the end of every cycle
    kernel_param_values(i+1,1) = kernel_param_c(1);
    fs(i+1) = fc;
    % Lower the temperature for next cycle
    t = frac * t;
end
% print solution
disp(['Best solution: ',num2str(kernel_param_c)])
disp(['Best objective: ',num2str(fc)])
% plot(kernel_param_values(:,1),kernel_param_values(:,2),'r-o')


fig = figure(2);
subplot(2,1,1)
plot(fs,'r.-')
legend('Objective')
subplot(2,1,2)
hold on
plot(kernel_param_values(:,1),'b.-')
legend('x_1')
