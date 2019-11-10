function []=update_ctrl(vol_h, img_h, delta, max_s)
%*
% * % * Copyright (C) {2018} Texas Instruments Incorporated - http://www.ti.com/ 
% * ALL RIGHTS RESERVED 
% * 
% * Updates visualization of gesture to control volume or an image zoom
% * 
% */   
global mri_data;

current = max(vol_h.YData);
if(isempty(current))
    current = 0;
end

new_ind = current - delta;
img_ind = new_ind;
if(new_ind <= 0)
	new_ind=0;
    img_ind = 1;
elseif(new_ind >= max_s)
    new_ind = max_s;
    img_ind = new_ind;
else
    new_ind = floor(new_ind);
    img_ind = new_ind;
end

%update volume control
set(vol_h, 'YData', 1:new_ind)

%update image control
set(img_h, 'CData', mri_data(:,:,img_ind))

end
