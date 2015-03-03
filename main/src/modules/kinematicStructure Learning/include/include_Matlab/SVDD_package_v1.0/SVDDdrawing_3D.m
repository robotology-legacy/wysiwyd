function SVDDdrawing_3D(f,class_idx,data_min,data_max,cVec)

warning off MATLAB:colon:operandsNotRealScalar

resol_1 = 50;%100;%200;
resol_2 = 10;%20;
resol_3 = 100;%200;%400;
thres_1 = 0.001;
thres_2 = 0.001;

ff = inline(f);

figure(100)
hold on
grid on

diff = data_max - data_min
diff_buf = diff/2;

x_range = data_min(1)-diff_buf(1):diff(1)/resol_1:data_max(1)+diff_buf(1);
y_range = data_min(2)-diff_buf(2):diff(2)/resol_1:data_max(2)+diff_buf(2);
z_range = data_min(3)-diff_buf(3):diff(3)/resol_1:data_max(3)+diff_buf(3);

x_range = x_range';
y_range = y_range';
z_range = z_range';

x_num = size(x_range,1);
y_num = size(y_range,1);
z_num = size(z_range,1);

in1 = repmat(x_range,y_num*z_num,1);
in1 = sort(in1,1);
in2 = repmat(y_range,z_num,1);
in2 = sort(in2,1);
in2 = repmat(in2,x_num,1);
in3 = repmat(z_range,x_num*y_num,1);

in = [in1,in2,in3];

result = abs(feval(ff,in(:,1),in(:,2),in(:,3)));

idx = find(result<thres_1);
plot3(in(idx,1),in(idx,2),in(idx,3),[cVec(class_idx+1) '.'],'MarkerSize',0.1);

for y_value = data_min(2)-diff_buf(2):diff(2)/resol_2:data_max(2)+diff_buf(2)
    x_value = data_min(1)-diff_buf(1):diff(1)/resol_3:data_max(1)+diff_buf(1);
    z_value = data_min(3)-diff_buf(3):diff(3)/resol_3:data_max(3)+diff_buf(3);    
    
    x_value = x_value';
    z_value = z_value';    
    
    x_num = size(x_value,1);
    z_num = size(z_value,1);    
    
    in2 = ones(x_num * z_num,1)*y_value;
    in1 = repmat(x_value,z_num,1);
    in3 = in1;
    in1 = sort(in1,1);
    
    in = [in1,in2,in3];
    
    result = abs(feval(ff,in(:,1),in(:,2),in(:,3)));
    
    idx = find(result<thres_2);
    
    odd_idx = idx([1:2:end]);
    even_idx = idx([2:2:end]);
    idx = [odd_idx;even_idx];
    
    plot3(in(idx,1),in(idx,2),in(idx,3),'k.','MarkerSize',0.5);      
end

for z_value = data_min(3)-diff_buf(3):diff(3)/resol_2:data_max(3)+diff_buf(3)
    x_value = data_min(1)-diff_buf(1):diff(1)/resol_3:data_max(1)+diff_buf(1);
    y_value = data_min(2)-diff_buf(2):diff(2)/resol_3:data_max(2)+diff_buf(2);
    
    x_value = x_value';
    y_value = y_value';
    
    x_num = size(x_value,1);
    y_num = size(y_value,1);    
    
    in3 = ones(x_num*y_num,1)*z_value;
    in1 = repmat(x_value,y_num,1);
    in2 = in1;
    in1 = sort(in1,1);
    
    in = [in1,in2,in3];
    
    result = abs(feval(ff,in(:,1),in(:,2),in(:,3)));
    
    idx = find(result<thres_2);
    
    odd_idx = idx([1:2:end]);
    even_idx = idx([2:2:end]);
    idx = [odd_idx;even_idx];
    
    plot3(in(idx,1),in(idx,2),in(idx,3),'r.','MarkerSize',0.5);      
end