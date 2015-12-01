%%
% calculate minimun radius
% exy_rad_buf = zeros(num_frames,1);
exy_rad_buf = [];
for frm_idx = 1:num_frames
    buf = [];
    for i = 1:size(full_exy{frm_idx},2)
        buf = [buf,round(sqrt(full_radius_map(full_exy{frm_idx}(2,i),full_exy{frm_idx}(1,i),frm_idx)))];
    end        
%     exy_rad_buf(frm_idx) = min(buf);
    exy_rad_buf = [exy_rad_buf,min(buf)];
end
% min_rad = min(exy_rad_buf);
min_rad = ceil(mean(exy_rad_buf))
disp(['Minimum radius: ',num2str(min_rad)]);

