function [] = plot_gesture_twirl(port_num)
%*
% * % * Copyright (C) {2018} Texas Instruments Incorporated - http://www.ti.com/ 
% * ALL RIGHTS RESERVED 
% * 
% * Plots radar data and determines whether twirl gesture occured
% * 
% */   

%% declare variables
global datavec;
global framecou;
global GESTURE_PKT_SIZE_BYTES;
global NUM_PKTS_TO_COLLECT;
global data_log;
global allbytes;

datavec = [];
framecou = 0;
GESTURE_PKT_SIZE_BYTES=26;
NUM_PKTS_TO_COLLECT =4;
data_log=[];
allbytes = [];

% Data stream
BUFFER_SIZE = 100;
x_sample = 1:BUFFER_SIZE;
numdetections = zeros(1,BUFFER_SIZE);
dopplerave = zeros(1,BUFFER_SIZE);
rangeave = zeros(1,BUFFER_SIZE);
angleval = zeros(1,BUFFER_SIZE);
rangeidx = zeros(1,BUFFER_SIZE);
magsum = zeros(1,BUFFER_SIZE);
numdetectionspos = zeros(1,BUFFER_SIZE);
doppleravepos = zeros(1,BUFFER_SIZE);
numdetectionsneg = zeros(1,BUFFER_SIZE);
doppleraveneg = zeros(1,BUFFER_SIZE);

% COM port
sphandle = configureSport(port_num);

% NN
L_training = 10;


% Visualization
load mri_exampledata.mat;
global mri_data;
mri_data = mri_subset;





%% Initialize Figures
h = figure(1);
set(gcf, 'color','w','units','normalized','OuterPosition',[0 0 0.5 1],'MenuBar','none','ToolBar','none');
%suptitle('Gesture: Twirl finger CCW vs CW')
t = annotation('textbox', 'String','Gesture: Twirl Finger CW or CCW', 'units','normalized', 'position', [0.05 0.9 0.85 0.1]);
t.LineStyle = 'none';
t.FontUnits = 'normalized';
t.FontSize = 0.025;

% instructions image
subplot(3,1,1); imshow('twirl_inst_pic.png', 'Border', 'tight')

% active range doppler
subplot(3,1,2);
p1 = plot(x_sample, numdetectionspos, 'b'); hold on; 
p2 = plot(x_sample, numdetectionsneg,'r' );
axis manual; ylim([0 100]); title('Active Bins in Range-Doppler'); grid on; hold off;

% doppler
subplot(3,1,3);
p3 = plot(x_sample, doppleravepos, 'b'); hold on;
p4 = plot(x_sample, doppleraveneg, 'r');
axis manual; ylim([0 20]); title('Weighted Doppler Signature'); grid on;


% Example applications figure
h_fig_apps = figure(2);
set(h_fig_apps, 'color','w','units','normalized','OuterPosition',[0.5 0 0.5 1],'MenuBar','none','ToolBar','none');
note1 = annotation('textbox', 'String','Example Applications:', 'units','normalized', 'position', [0.05 0.9 0.85 0.1]);
note1.LineStyle = 'none';
note1.FontUnits = 'normalized';
note1.FontSize = 0.025;

% image control plot
subplot('position', [0.05 0.325 0.9 0.6]),
img_h = imshow(mri_data(:,:,1)); colormap(map);
title('Image Zoom Control')
set(img_h.Parent, 'xlimmode','manual',...
'ylimmode','manual',...
'zlimmode','manual',...
'climmode','manual',...
'alimmode','manual',...
'CLim', [cm_min cm_max]);

% volume control plot
vol_max = length(mri_data(1,1,:));
subplot('position', [0.05 0.05 0.9 0.25]),
empty_h = bar([1:vol_max],'FaceColor','none'); axis tight; hold on;
vol_h = bar(zeros(1,vol_max),'FaceColor','blue'); axis manual; hold off;
title('Volume Control')
set(vol_h.Parent, 'XTickLabel', {}, 'YTickLabel', {})
set(vol_h, 'YData', 1:3)


%% Run
while (ishghandle(h)) 
   %disp(get(sphandle,'BytesAvailable'));
   if ~isempty(datavec)
       if (datavec(1)==11) % check magic word
           datavec1=reshape(datavec,GESTURE_PKT_SIZE_BYTES/2,NUM_PKTS_TO_COLLECT);
	
           numdetections = [numdetections(1+NUM_PKTS_TO_COLLECT:end) datavec1(2,:) ]; %numdetections
           dopplerave =[dopplerave(1+NUM_PKTS_TO_COLLECT:end) datavec1(3,:) ]; %dopplerave
           rangeave=[rangeave(1+NUM_PKTS_TO_COLLECT:end) datavec1(4,:) ]; %rangeave
           magsum=[magsum(1+NUM_PKTS_TO_COLLECT:end) datavec1(5,:) ]; %angleval
           angleval=[angleval(1+NUM_PKTS_TO_COLLECT:end) datavec1(11,:) ]; %angleval
           rangeidx=[rangeidx(1+NUM_PKTS_TO_COLLECT:end) datavec1(7,:) ]; %

           numdetectionspos=[numdetectionspos(1+NUM_PKTS_TO_COLLECT:end) datavec1(5,:) ]; %numdetections
           doppleravepos=[doppleravepos(1+NUM_PKTS_TO_COLLECT:end) datavec1(6,:) ]; %dopplerave

           numdetectionsneg=[numdetectionsneg(1+NUM_PKTS_TO_COLLECT:end) datavec1(8,:) ]; %numdetections
           doppleraveneg=[doppleraveneg(1+NUM_PKTS_TO_COLLECT:end) datavec1(9,:) ]; %dopplerave
           
           % format data for classifier
           xin=[numdetectionspos([end-L_training+1:end]) numdetectionsneg([end-L_training+1:end]) doppleravepos([end-L_training+1:end]) doppleraveneg([end-L_training+1:end]) rangeave([end-L_training+1:end]) ];
           
           % *****classification***** 
           xout=my_VolCtrlVsFineTune_nn(xin'); 
	
           % determine control change inc/dec
           xout_vol=0;
		   L1=10;
		   angleval_temp=angleval([end-L1:end])-mean(angleval([end-L1:end]));
		   corr1=sum(angleval_temp.*numdetectionspos([end-L1:end]));
		   corr2=sum(angleval_temp.*numdetectionsneg([end-L1:end]));
		   C1=(sum(numdetectionspos([end-L1:end]))>20)&&(sum(numdetectionsneg([end-L1:end]))>20);
	
		   if((C1)&&(xout(1)>xout(2)))
			   if((corr1>corr2))
				xout_vol=1;
			   elseif((corr1<corr2))
				xout_vol=-1;
			   else
				xout_vol=0;
               end
           end
		   
           % *****plotting***** 
           % update plots w/ incoming data
           set(p1, 'YData', numdetectionspos);
           set(p2, 'YData', numdetectionsneg);
           set(p3, 'YData', doppleravepos);
           set(p4, 'YData', abs(doppleraveneg));
           
           % plot application examples 
           update_ctrl(vol_h, img_h, xout_vol, vol_max)
           
           % clear data
           datavec=[];
       else
          display('missing magic word');
          %%uiwait(msgbox(sprintf('Lost sync of serial port data(%d)',get(sphandle,'BytesAvailable')),'Error','modal'));
          numextra = get(sphandle,'BytesAvailable');
          if numextra > 0
            fread(sphandle,numextra,'uint8');
          end
          %sphandle = configureSport();
          datavec = [];
          %title(sprintf('Serial port sync loss!  Frame number = %d',framecou));
          drawnow;
        end % magic word
    end % empty data vec
    pause(0.02);
    
end % while

% close and delete COM ports
fclose(sphandle);
delete(sphandle);
clear sphandle;

return % fcn

%% Data handling
function [] = plotImage(obj, event)
global GESTURE_PKT_SIZE_BYTES
global NUM_PKTS_TO_COLLECT
global datavec;
global framecou;

datavec = fread(obj, GESTURE_PKT_SIZE_BYTES*NUM_PKTS_TO_COLLECT/2, 'int16');
framecou = framecou + 1;

return

%% Helper 
function [] = dispError()
disp('error!');
return

function [sphandle] = configureSport(port_num)
    global GESTURE_PKT_SIZE_BYTES
    global NUM_PKTS_TO_COLLECT;
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    sphandle = serial(['COM' num2str(port_num)],'BaudRate',921600);
    set(sphandle,'InputBufferSize',GESTURE_PKT_SIZE_BYTES*NUM_PKTS_TO_COLLECT);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    set(sphandle,'BytesAvailableFcnMode','byte');
    set(sphandle,'BytesAvailableFcnCount',GESTURE_PKT_SIZE_BYTES*NUM_PKTS_TO_COLLECT);
    set(sphandle,'BytesAvailableFcn',@plotImage);
    fopen(sphandle);

return
