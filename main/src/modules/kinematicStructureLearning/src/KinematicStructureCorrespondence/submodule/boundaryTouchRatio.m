
function out = boundaryTouchRatio(mask, height, width)

    touch_direction = zeros(1,4);
    connection_thres = 0.1;
    background_thres = 0.5;
    
    %--------------------------------------
    buf = 0;
    h=1;
    for w=1:width-1
        if mask(h,w) == 1
            buf = buf + 1;
        end
    end
    ratio = (buf / width);
    if ratio >= connection_thres && ratio < background_thres
        touch_direction(1,1) = 1;
    elseif ratio >= background_thres
        touch_direction(1,1) = 2;
    end

    %--------------------------------------
    buf = 0;
    w=width;
    for h=1:height-1
        if mask(h,w) == 1
            buf = buf + 1;
        end        
    end    
    
    ratio = (buf / height);
    if ratio >= connection_thres && ratio < background_thres
        touch_direction(1,2) = 1;
    elseif ratio >= background_thres
        touch_direction(1,2) = 2;
    end    

    %--------------------------------------
    buf = 0;
    h=height;
    for w=width:-1:2
        if mask(h,w) == 1
            buf = buf + 1;
        end        
    end
    ratio = (buf / width);
    if ratio >= connection_thres && ratio < background_thres
        touch_direction(1,3) = 1;
    elseif ratio >= background_thres
        touch_direction(1,3) = 2;
    end     
    
    %--------------------------------------
    buf = 0;
    w=1;
    for h=height:-1:2
        if mask(h,w) == 1
            buf = buf + 1;
        end        
    end
    ratio = (buf / height);
    if ratio >= connection_thres && ratio < background_thres
        touch_direction(1,4) = 1;
    elseif ratio >= background_thres
        touch_direction(1,4) = 2;
    end    
    
%     touch_direction
    %--------------------------------------
    
    if length(find(touch_direction==0)) == 4
%         out = 'Isolated';
        out = 0;
    elseif length(find(touch_direction==0)) == 3 && length(find(touch_direction==1)) == 1
%         out = 'Connected';
        out = 1;
    else
%         out = 'Background';
        out = 2;
    end
end
