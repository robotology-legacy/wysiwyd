
num_step = 50;
refm = rotx(0);
dist_buf = zeros(num_step,num_step);
for i = 1:num_step
    theta_x = (i-1)*pi/num_step;
    cmx = rotx(theta_x);
    for j = 1:num_step
        theta_y = (j-1)*pi/num_step;
        cmy = roty(theta_y);
        cm = cmx*cmy;
        dist = norm(logm(refm'*cm),'fro');
        dist_buf(i,j) = dist;
    end
end

figure
surf(dist_buf)