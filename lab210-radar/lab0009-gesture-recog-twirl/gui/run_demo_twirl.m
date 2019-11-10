%*
% * % * Copyright (C) {2018} Texas Instruments Incorporated - http://www.ti.com/ 
% * ALL RIGHTS RESERVED 
% * 
% * Runs demo for twirl gesture
% * 
% */   

clear, clc, close all

%% get COM ports
ports = get_com_ports();
portStr = {};
for i =1:length(ports(:,1))
    portStr{end+1} = ['COM ' num2str(ports(i,1)) ' and ' num2str(ports(i,2))];
end
       
       
[s,ok] = listdlg('PromptString','Select IWR14XX EVM ports:',...
               'SelectionMode','single',...
               'ListString', portStr)

if(ok)           
    %% load config
    set(0,'DefaultFigureVisible','off')
    mmw_demo('xwr14xx', ports(s,2) , 1, 0.5, ports(s,1), 'profile_2d_gesture_128.cfg', 1)
    set(0,'DefaultFigureVisible','on')
    close all
    %% run demo
    plot_gesture_twirl( ports(s,2))
else
    error('COM port not selected. Exiting')
end