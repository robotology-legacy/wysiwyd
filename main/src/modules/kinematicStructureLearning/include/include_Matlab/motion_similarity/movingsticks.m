function dist = movingsticks(alpha_start, alpha_end, ...
    beta_start, beta_end, ...
    gamma_start, gamma_end, ...
    F)
numSticks = 4;
sticks = zeros(2, numSticks, F);

l = 5;

f = 1 : F;
alpha = (alpha_end-alpha_start)*f/F + alpha_start;
beta  = (beta_end - beta_start)*f/F + beta_start;
gamma = (gamma_end-gamma_start)*f/F + gamma_start;

for f = 1:F
    sticks(:,2,f) = [-3 0];
    sticks(:,3,f) = sticks(:,2,f)' + [l * cosd(gamma(f)) l * sind(gamma(f))];
    
    sticks(:,1,f) = sticks(:,2,f)' + [l * cosd(alpha(f)) l * sind(alpha(f))];
    sticks(:,4,f) = sticks(:,3,f)' + [l * cosd(beta(f)) l  * sind(beta(f))];
end

minxy = min(min(min(sticks(:,:,:))));
maxxy = max(max(max(sticks(:,:,:))));

fig = figure();
%set(fig,'Name','Movement 1');

alpha_rad = alpha * 2 * pi / 360;
beta_rad = beta * 2 * pi / 360;
gamma_rad = gamma * 2 * pi / 360;

dist = zeros(F,3);

for f = 1:F
%     clf;
    hold on;
    plot(sticks(1,:,f),sticks(2,:,f),'bo');
    plot([sticks(1,1,f),sticks(1,2,f)],[sticks(2,1,f),sticks(2,2,f)],'Color',[0.5-(f/F)/2,1,1]);
    plot([sticks(1,2,f),sticks(1,3,f)],[sticks(2,2,f),sticks(2,3,f)],'Color',[0.5-(f/F)/2,1,1]);
    plot([sticks(1,3,f),sticks(1,4,f)],[sticks(2,3,f),sticks(2,4,f)],'Color',[0.5-(f/F)/2,1,1]);
    axis([minxy,maxxy,minxy,maxxy]);
    axis equal
    pause(0.005);
    
    alpha_rotz_f = rotz(alpha_rad(f));
    beta_rotz_f = rotz(beta_rad(f));
    gamma_rotz_f = rotz(gamma_rad(f));

    % alpha-beta pair / beta-gamma pair / gamma-alpha pair
    pair_buf = [cal_logm(alpha_rotz_f, beta_rotz_f), cal_logm(beta_rotz_f, gamma_rotz_f), cal_logm(gamma_rotz_f, alpha_rotz_f)];
    dist(f,:) = pair_buf;
    
end
end