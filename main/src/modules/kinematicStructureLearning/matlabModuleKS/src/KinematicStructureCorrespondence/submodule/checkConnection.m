function connection = checkConnection(KineStruct,idx)

%%
% idx = 'P';
cdata.filename = 'video.avi';
cdata.pathname = ['data/KSC/',idx,'/'];
% disp([cdata.pathname,cdata.filename]);
videoObj = VideoReader([cdata.pathname,cdata.filename]);
cdata.nFrames = videoObj.NumberOfFrames;
cdata.width = videoObj.Width;
cdata.height = videoObj.Height;

%%
connection_check_idx = zeros(cdata.nFrames,KineStruct.num_seg);

for frm_idx = 1:cdata.nFrames
    img = read(videoObj,frm_idx);
    img_gray = rgb2gray(img);
    
        I = im2double(adapthisteq(img_gray));
    
    for seg_idx = 1:KineStruct.num_seg
        
        y = round(KineStruct.seg_center(1,seg_idx,frm_idx));
        x = round(KineStruct.seg_center(2,seg_idx,frm_idx));
        
        J = regiongrowing(I,x,y,0.1);
        
        out = boundaryTouchRatio(J, cdata.height, cdata.width);
       
        connection_check_idx(frm_idx,seg_idx) = out;
    end
end

connection_thres = 0.8;

connection = [-1,-1,-1];
for seg_idx = 1:KineStruct.num_seg
    if length(find(connection_check_idx(:,seg_idx) == 1)) >= connection_thres
        connection(seg_idx) = 1;
    else
        connection(seg_idx) = 0;
    end
end

end