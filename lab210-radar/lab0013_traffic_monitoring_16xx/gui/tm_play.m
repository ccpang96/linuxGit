colors = 'bgrcm';

% Point Cloud Source choices: {'adcCaptured' | 'pointCloudCaptured' | 'pointCloudTV'};
PointCloudDataSource = 'pointCloudCaptured';
%pointCloudFilePath = '\\gtda0869680a\shared\Mar6_GTtest2_normVmax_over3Vmax\';
pointCloudFilePath = '.\';
pointCloudFileName = [pointCloudFilePath, 'fhistRT'];
%{
pointCloudFilePath = '\\gtda0869680a\shared\TMv2_phaseCorrectionComparison\adc_samples_tm_test_h5m_1car_t2_bothTxRx\';
pointCloudFileName = [pointCloudFilePath,'adc_samples_tm_test_h5m_1car_t2_aoaResults.mat'];
%}

% Scene Run choices: {'DallasParkingLot' | 'GTParkingLot1' | GTParkingLot2};
sceneRun = 'GTParkingLot1';
% Tracker Run choice: {'Target' | 'PCMex' | 'PCMatlab'};
trackerRun = 'PCMatlab';

captureVideo = 0;

if(strcmp(trackerRun,'PCMex') || strcmp(trackerRun,'PCMatlab'))
    GTRACK_PATH = 'C:\Radar\top\mmwave_sdk\ti\alg\gtrack';
    GTRACK_MEX_PATH = [GTRACK_PATH,'\test\win\matlab\mex'];
    GTRACK_MATLAB_PATH = [GTRACK_PATH,'\test\win\matlab\src'];
    path(path,GTRACK_MEX_PATH);
    path(path,GTRACK_MATLAB_PATH);
end
%[controlSerialPort, dataSerialPort, configurationFileName, loadCfg] = configDialog();
controlSerialPort = 3;
dataSerialPort = 4;
configurationFileName = 'mmw_tm_demo_ph2.cfg';
loadCfg = 0;
mmwDemoCliPrompt = char('mmwDemo:/>');

% ************************************************************************
% Desktop setup
% We create custom group, and add tiling figures into that group
desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
myIcon = javax.swing.ImageIcon('texas_instruments.gif');
myGroupName = 'Texas Instruments - IWR16xx Traffic Monitoring Demo Visualization';
myGroup = desktop.addGroup(myGroupName);
desktop.setGroupDocked(myGroupName, 0);
myDim = java.awt.Dimension(5, 3);   % Tiling 3x5 windows [1 2 3 4 5; 6 7 8 9 10; 11 12 13 14 15];
desktop.setDocumentArrangement(myGroupName, 2, myDim);
% We combine second column windows 2,7, and 12 into one window
desktop.setDocumentRowSpan(myGroupName, 0, 1, 3); %Row 0,Col 1 spans 3 Rows
% We combine third column windows 3,8, and 13 into one window
desktop.setDocumentRowSpan(myGroupName, 0, 2, 3); %Row 0,Col 2 spans 3 Rows
% We combine forth column windows 4,9, and 14 into one window
desktop.setDocumentRowSpan(myGroupName, 0, 3, 3); %Row 0,Col 3 spans 3 Rows
% We combine forth column windows 5,10, and 15 into one window
desktop.setDocumentRowSpan(myGroupName, 0, 4, 3); %Row 0,Col 3 spans 3 Rows
% Resulted tiling structure is:
% [1] |2| |3| |4| |5|
% [6] | | | | | | | |
% [7] | | | | | | | |

% First row is narrow, second and third rows are equal
desktop.setDocumentColumnWidths(myGroupName, [0.14 0.21 0.21 0.18 0.26]);
desktop.setDocumentRowHeights(myGroupName, [0.3 0.6 0.1]);
pause(1);  % Must yield for java process to execute

figureTitles = {'Statistics', 'Point Cloud', 'Gating and Association', 'Ground Truth', 'Cum Cloud', 'Chirp Configuration', 'Control', 'Doppler Map'};
figureTiles = [1, 2, 3, 4, 5, 6, 7, 4];
numFigures = size(figureTitles, 2);
hFigure = zeros(1,numFigures);
matlabFileName = 'vehicleCountMovie';

%{
videoFile = 'groundTruthGT.mp4';
videoObj = VideoReader(videoFile);
videoObj.CurrentTime = 6;
load('cameraParam.mat', 'cameraParams');
%}

warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame;
for iFig = 1:numFigures
    % Place a figure to a tile
    if(iFig ~= figureTiles(iFig))
        figure(hFigure(figureTiles(iFig)));
    end
    hFigure(iFig) = figure('WindowStyle', 'docked', ...
            'Name', figureTitles{iFig}, ...
            'NumberTitle', 'off'); 
    set(get(handle(hFigure(iFig)), 'javaframe'),'GroupName', myGroupName);
    pause(0.1);  % Must yield for java process to execute
end

% get the rendered width and height of myGroup
frameWidth=myGroup.getInternalFrame.getVisibleRect.width;
frameHeight=myGroup.getInternalFrame.getVisibleRect.height;
bufferedImage=myGroup.getInternalFrame.createImage(frameWidth,frameHeight);
% get the JPanel
panel=myGroup.getInternalFrame.getComponent(0);
% print it into the bufferedImage
panel.print(bufferedImage.getGraphics());

% Setup tracking scene
%scene parameters
% Dallas Paking Lot
if(strcmp(sceneRun,'GTParkingLot1'))
    scene.numberOfLanes = 3;
    scene.laneWidth = [3 3 3];
    scene.leftLineX = -5;
    scene.lineX = [scene.leftLineX scene.leftLineX+cumsum(scene.laneWidth)];
    scene.stopLineY = [21 20 20 20];
        
    scene.maxPos = [scene.lineX(1)-10 scene.lineX(end)+10 0 80];
    scene.numberOfTargetBoxes = scene.numberOfLanes;
    scene.targetBox = [scene.lineX(1:end-1); scene.stopLineY(1:end-1); scene.laneWidth; 50-scene.stopLineY(1:end-1)]';

    scene.numberOfBoundaryBoxes = 1;
    scene.boundaryBox(1,:) = [scene.lineX(1)-2 15 scene.lineX(end)-scene.lineX(1)+4 60]; %left, bottom, width, height
    
    scene.numberOfStaticBoxes = 1;
    scene.staticBox(1,:) = [scene.lineX(1)-1 19 scene.lineX(end)-scene.lineX(1)+2 31]; %left, bottom, width, height
    
%    scene.azimuthTilt = 0; %4 degrees horizontal tilt
    scene.azimuthTilt = 4*pi/180; %4 degrees horizontal tilt
%    scene.azimuthTilt = +11*pi/180; %4 degrees horizontal tilt
end

if(strcmp(sceneRun,'DallasParkingLot'))
    scene.numberOfLanes = 3;
    scene.laneWidth = [3 3 3];
    scene.leftLineX = 2.8;
    scene.lineX = [scene.leftLineX scene.leftLineX+cumsum(scene.laneWidth)];
    scene.stopLineY = [18 18 18 18];
        
    scene.maxPos = [scene.lineX(1)-10 scene.lineX(end)+10 0 80];
    scene.numberOfTargetBoxes = scene.numberOfLanes;
    scene.targetBox = [scene.lineX(1:end-1); scene.stopLineY(1:end-1); scene.laneWidth; 50-scene.stopLineY(1:end-1)]';

    scene.numberOfBoundaryBoxes = 1;
    scene.boundaryBox(1,:) = [scene.lineX(1)-2 15 scene.lineX(end)-scene.lineX(1)+4 60]; %left, bottom, width, height
    
    scene.numberOfStaticBoxes = 1;
    scene.staticBox(1,:) = [scene.lineX(1)-1 19 scene.lineX(end)-scene.lineX(1)+2 31]; %left, bottom, width, height
    
%    scene.azimuthTilt = 0; %4 degrees horizontal tilt
    scene.azimuthTilt = 6*pi/180; %4 degrees horizontal tilt
%    scene.azimuthTilt = -4*pi/180; %4 degrees horizontal tilt
%    scene.azimuthTilt = +11*pi/180; %4 degrees horizontal tilt
end

%Read Chirp Configuration file
cliCfg = readCfg(configurationFileName);
Params = parseCfg(cliCfg);

%sensor parameters
sensor.rangeMax = 74;
sensor.rangeMin = 10;
sensor.azimuthFoV = 90*pi/180; %120 degree FOV in horizontal direction
sensor.framePeriod = Params.frameCfg.framePeriodicity; %in ms
sensor.maxRadialVelocity = Params.dataPath.dopplerResolutionMps*15;
sensor.radialVelocityResolution = Params.dataPath.dopplerResolutionMps;
sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);
        
S = warning('off', 'MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
trackingAx = [];
gatingAx = [];
dopplerAx = [];
cumCloudAx = [];
groundTruthAx = [];
hStatGlobal = [];

%Ground truth data
%{
load('gtruth.mat', 'gtruth');
gtruthFrame(:,1) = gtruth(1,1):gtruth(end,1);
gtruthFrame(:,2) = round(interp1(gtruth(:,1),gtruth(:,2),gtruthFrame(:,1)));
gtruthFrame(:,3) = round(interp1(gtruth(:,1),gtruth(:,3),gtruthFrame(:,1)));
gtruthFrame(:,4) = round(interp1(gtruth(:,1),gtruth(:,4),gtruthFrame(:,1)));
gtImagePoints= [gtruthFrame(:,2),gtruthFrame(:,3)];

load('cameraParam.mat', 'cameraParams');
vidFrame = readFrame(videoObj);
figure;
imshow(vidFrame);
imagePoints = [ 
    699 658; 690 711; 679 784; 665 882; 642 1029;
    1086 664; 1136 721; 1200 794; 1299 901; 1447 1063];
worldPoints = [
    0 0; 0 260; 0 520; 0 780; 0 1040;
    490 0; 490 260; 490 520; 490 780; 490 1040];    
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
hold on;
plot(imagePoints(:,1), imagePoints(:,2),  'o');

worldImagePoints = pointsToWorld(cameraParams, R, t, gtImagePoints);
%}

for iFig = 1:8
    figure(hFigure(iFig));
    if(strcmp(figureTitles{iFig},'Point Cloud') || strcmp(figureTitles{iFig},'Gating and Association'))
        plot(sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
        plot([0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');
        xlabel('X coordinate, m');
        ylabel('Y coordinate, m');
        axis equal;
        axis(scene.maxPos);
        
        for n=1:scene.numberOfBoundaryBoxes
            rectangle('Position', scene.boundaryBox(n,:), 'EdgeColor','g', 'LineStyle', '-', 'LineWidth', 0.5);
        end
        for n=1:scene.numberOfStaticBoxes
            rectangle('Position', scene.staticBox(n,:), 'EdgeColor','m', 'LineStyle', '-', 'LineWidth', 0.5);
        end
        
        for xLine = 1:length(scene.lineX)
            if((xLine == 1) || (xLine == length(scene.lineX)))
                line('Xdata',[scene.lineX(xLine) scene.lineX(xLine)],'YData',[scene.stopLineY(xLine) 80], 'Color','k', 'LineStyle', '-', 'LineWidth', 1);
            else
                line('Xdata',[scene.lineX(xLine) scene.lineX(xLine)],'YData',[scene.stopLineY(xLine) 80], 'Color','k', 'LineStyle', '--');
            end
        end
        for stopLine=1:scene.numberOfLanes   
            line('Xdata',[scene.lineX(stopLine) scene.lineX(stopLine+1)],'YData',[scene.stopLineY(stopLine) scene.stopLineY(stopLine)], 'Color','r', 'LineStyle', '-', 'LineWidth', 2);
        end
        
        grid on;
        hold on;
        grid minor;
        
        if(strcmp(figureTitles{iFig},'Point Cloud'))
            trackingAx = gca;
        end
        if(strcmp(figureTitles{iFig},'Gating and Association'))
            gatingAx = gca;
        end
    end
    
    if(strcmp(figureTitles{iFig},'Ground Truth'))
        groundTruthAx = gca;
    end
    
    if(strcmp(figureTitles{iFig},'Chirp Configuration'))
        tablePosition = [0.05 0.05 0.9 0.9];
        h = displayChirpParams(Params, tablePosition);
        h.InnerPosition = [h.InnerPosition(1:2) h.Extent(3:4)];
    end
    
    if(strcmp(figureTitles{iFig},'Statistics'))
        hStatGlobal(1) = text(0, 0.9, 'Frame # 0', 'FontSize',12);
        hStatGlobal(2) = text(0, 0.8, 'Detection Points: 0','FontSize',12);
        hStatGlobal(3) = text(0, 0.5, 'Target Count:  0','FontSize',14);
        hStatGlobal(4) = text(0, 0.4, 'In Box Count:  0','FontSize',14);
        axis off;
    end
    
    if(strcmp(figureTitles{iFig},'Control'))
        cFig = iFig;
        hRbPause = uicontrol(hFigure(cFig),'Style','radio','String','Pause',...
            'Position',[180 10 50 20],'Value',0);
        hPbExit = uicontrol(hFigure(cFig),'Style', 'pushbutton', 'String', 'Exit',...
            'Position', [10 10 50 20],'Callback', @exitPressFcn);
        setappdata(hPbExit, 'exitKeyPressed', 0);
        
        hPbStep = uicontrol(hFigure(cFig),'Style', 'pushbutton', 'String', 'Step',...
            'Position', [100 10 50 20],'Callback', @stepPressFcn);
        setappdata(hPbStep, 'stepKeyPressed', 0);
    end
    
    if(strcmp(figureTitles{iFig},'Doppler Map'))
        setDopplerMapFigure(sensor);
        hPbClearDoppler = uicontrol(hFigure(iFig),'Style', 'pushbutton', 'String', 'Clear',...
            'Position', [10 10 50 20],'Callback', @clearDopplerPressFcn);
        setappdata(hPbClearDoppler, 'sensor', sensor);        
        dopplerAx = gca;
    end
    if(strcmp(figureTitles{iFig},'Cum Cloud'))
        setCumCloudFigure(sensor, scene);
        hPbClearCloud = uicontrol(hFigure(iFig),'Style', 'pushbutton', 'String', 'Clear',...
            'Position', [10 10 50 20],'Callback', @clearCloudPressFcn);
        setappdata(hPbClearCloud, 'sensor', sensor);        
        setappdata(hPbClearCloud, 'scene', scene);        
        cumCloudAx = gca;
    end   
end

maxNumTracks = 20;
maxNumPoints = 250;

hPlotCloudHandleAll = [];
hPlotCloudHandleOutRange = [];
hPlotCloudHandleClutter = [];
hPlotCloudHandleStatic = [];
hPlotCloudHandleDynamic =[];
hPlotPoints3D = [];

hTargetBoxHandle = zeros(scene.numberOfTargetBoxes,1);

clutterPoints = zeros(2,1);
activeTracks = zeros(1, maxNumTracks);

trackingHistStruct = struct('tid', 0, 'allocationTime', 0, 'tick', 0, 'posIndex', 0, 'histIndex', 0, 'sHat', zeros(1000,6), 'ec', zeros(1000,9),'pos', zeros(100,2), 'hMeshU', [], 'hMeshG', [], 'hPlotAssociatedPoints', [], 'hPlotTrack', [], 'hPlotCentroid', []);
trackingHist = repmat(trackingHistStruct, 1, maxNumTracks);

% If tracking is at PC, create tracking module
if(strcmp(trackerRun,'PCMex') || strcmp(trackerRun,'PCMatlab'))
    trackerConfig.stateVectorType = 1; % 0=>2D, 1=>2DA
    trackerConfig.maxNumPoints = maxNumPoints;
    trackerConfig.maxNumTracks = maxNumTracks;

    trackerConfig.maxRadialVelocity = sensor.maxRadialVelocity;
    trackerConfig.radialVelocityResolution = sensor.radialVelocityResolution;
    trackerConfig.maxAcceleration = [0 5];
    trackerConfig.deltaT = sensor.framePeriod/1000; %in sec
    trackerConfig.initialRadialVelocity = 0;    
    trackerConfig.ax = [];
    
    boundaryBoxes = [scene.boundaryBox(:,1), scene.boundaryBox(:,1)+scene.boundaryBox(:,3), scene.boundaryBox(:,2), scene.boundaryBox(:,2)+scene.boundaryBox(:,4)];
    staticBoxes = [scene.staticBox(:,1), scene.staticBox(:,1)+scene.staticBox(:,3), scene.staticBox(:,2), scene.staticBox(:,2)+scene.staticBox(:,4)];

    trackerConfig.advParams.scenery = [scene.numberOfBoundaryBoxes, boundaryBoxes scene.numberOfStaticBoxes, staticBoxes];
    trackerConfig.advParams.allocation = [250, 100, 1, 5, 2.8, 5*sensor.radialVelocityResolution]; % Total SNR, min Velocity, min Points, distance, velocity separation
    trackerConfig.advParams.gating = [16, 12,6,0]; % Volume, Length Width and Velocity Limits
    trackerConfig.advParams.thresholds = [3, 3, 5, 1000, 5]; % det2act, det2free, act2free, stat2free, exit2free
    trackerConfig.advParams.variations = [4/sqrt(12), 1.5/sqrt(12), 1]; % Height, Width, Doppler std
end

if(strcmp(trackerRun,'PCMex'))
    trackerConfig.verbose = 3;
    hRadarTrackingC = gtrack_create_mex(trackerConfig);
end
if(strcmp(trackerRun,'PCMatlab'))
    trackerConfig.verbose = 3;
    hRadarTrackingM = trackModule('trackerConfig', trackerConfig);
end

point3D = [];
videoFrame = struct('cdata',[],'colormap', []);
frameNum = 0;
fprintf('------------------\n');

pause(0.1);
panel = myGroup.getInternalFrame.getComponent(0);
panel.print(bufferedImage.getGraphics());
% now get the java RGB pixel values
rasta=bufferedImage.getRGB(0,0,frameWidth,frameHeight,[],0,frameWidth);
% convert to Matlab values
rasta=256^3+rasta;
BLUE=uint8(mod(rasta,256));
GREEN=uint8(mod((rasta-int32(BLUE))./256,256));
RED=uint8(mod((rasta-256*int32(GREEN))./65536,256));
% make a cdata struct
fGroup.cdata=uint8(zeros(frameHeight,frameWidth,3));
fGroup.cdata(:,:,1)=reshape(RED,[frameWidth frameHeight])';
fGroup.cdata(:,:,2)=reshape(GREEN,[frameWidth frameHeight])';
fGroup.cdata(:,:,3)=reshape(BLUE,[frameWidth frameHeight])';
fGroup.colormap=[];

lastFile = 0;
fileNum = 0;
previousPointCloud = zeros(4,0, 'single');

while ~lastFile 
    
    fileNum = fileNum + 1;
    
    switch PointCloudDataSource
        case 'adcCaptured'
            load(pointCloudFileName, 'angleEst');
            if exist(pointCloudFileName, 'file' )
                load(pointCloudFileName, 'angleEst');
                disp(['Loading data from ', pointCloudFileName, ' ...']);
            else
                break;
            end
            lastFile = 1;
            numFrames = size(angleEst,2);
        
            frameStatStruct = struct('targetFrameNum', [], 'bytes', [], 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, 'start', 0, 'benchmarks', [], 'done', 0, ...
            'pointCloud', [], 'targetList', [], 'indexArray', []);
            fHist = repmat(frameStatStruct, 1, numFrames);        
        
        case 'pointCloudTV'
        case 'pointCloudCaptured'    
            fHistFileName = [pointCloudFileName, '_', num2str(fileNum, '%04d'), '.mat'];
            if exist(fHistFileName, 'file' )
                load(fHistFileName,'fHist');
                disp(['Loading data from ', fHistFileName, ' ...']);
                numFrames = size(fHist,2);
            else
                break;
            end
    end

    fSnapShot = repmat(videoFrame, numFrames,1);    

    for frameNum=1:numFrames
        if(frameNum == 160)
            disp(frameNum);
        end
        if(get(hRbPause, 'Value') == 1)
            while(get(hRbPause, 'Value') == 1)
                if(getappdata(hPbStep, 'stepKeyPressed') == 1)
                    setappdata(hPbStep, 'stepKeyPressed', 0);
                    break;
                end
                pause(1);
            end
        end 
        
        if(strcmp(PointCloudDataSource,'adcCaptured'))
            % Play captured Cloud
            targetFrameNum = frameNum;
            numInputPoints = size(angleEst{frameNum},2);
            pointCloud = zeros(4,numInputPoints, 'single');
            if (numInputPoints > 0)
                for n=1:numInputPoints
                    pointCloud(:,n) = [angleEst{frameNum}(n).range; angleEst{frameNum}(n).angles*pi/180 + scene.azimuthTilt; angleEst{frameNum}(n).doppler; angleEst{frameNum}(n).estSNR];
                end
                posAll = [pointCloud(1,:).*sin(pointCloud(2,:)); pointCloud(1,:).*cos(pointCloud(2,:))];
                snrAll = pointCloud(4,:);
            else
                posAll = [];
                snrAll = [];
            end
            numOutputPoints = single(numInputPoints);
            
        elseif(strcmp(PointCloudDataSource,'pointCloudCaptured'))
            targetFrameNum = fHist(frameNum).targetFrameNum;
            numInputPoints = single(size(fHist(frameNum).pointCloud,2));
            numOutputPoints = numInputPoints;

            if(strcmp(trackerRun,'Target'))
                pointCloud = previousPointCloud;
                previousPointCloud = fHist(frameNum).pointCloud;
            else
                pointCloud = fHist(frameNum).pointCloud;
                if(~isempty(pointCloud))
                    pointCloud(2,:)= -pointCloud(2,:) + scene.azimuthTilt; 
                end
            end
            
            if isempty(pointCloud)
                posAll = [];
                snrAll = [];
            else
                posAll = [pointCloud(1,:).*sin(pointCloud(2,:)); pointCloud(1,:).*cos(pointCloud(2,:))];
                snrAll = pointCloud(4,:);
            end
            fHist(frameNum).numOutputPoints = numOutputPoints;
            fHist(frameNum).numInputPoints = numInputPoints;
        end
        
        % Delete previous points
        if(ishandle(hPlotCloudHandleAll))
            delete(hPlotCloudHandleAll);
        end
        if(size(posAll,2))
            % Plot all points
            hPlotCloudHandleAll = scatter(trackingAx, posAll(1,:), posAll(2,:),'.k','SizeData',snrAll*10);
            plot(dopplerAx, pointCloud(3,:), posAll(2,:), '.k');
            plot3(cumCloudAx, posAll(1,:), posAll(2,:), pointCloud(4,:),'.b');
        end
        
        switch trackerRun
            case 'PCMex'        
                [TID, S, EC, Gain, mIndex] = gtrack_step_mex(hRadarTrackingC, pointCloud, [], numOutputPoints);
                
                numTargets = length(TID);
                
                fHist(frameNum).numTargets = numTargets;
                fHist(frameNum).targetList.numTargets = numTargets;
                fHist(frameNum).targetList.TID = TID;
                fHist(frameNum).targetList.S = S;
                fHist(frameNum).targetList.EC = EC;
                fHist(frameNum).targetList.G = Gain;
                fHist(frameNum).indexArray = mIndex;                    
                    
            case 'PCMatlab'            
                [track, num, mIndex] = step(hRadarTrackingM, pointCloud, 0, numOutputPoints);
                %translate from Matlab to C
                TID = zeros(1,num);
                S = zeros(6,num);
                EC = zeros(9,num);
                for n=1:num
                    TID(n) = track(n).trackerId -1; % In Matlab, TID are one-based
                    S(:,n) = track(n).S;
                    EC(:,n) = reshape(track(n).ec, 9,1);
                    Gain(n) = track(n).g;
                end            
                mIndex = mIndex -1;

            case 'Target'
                if isempty(fHist(frameNum).header)
                    continue;
                end
                if isempty(fHist(frameNum).targetList)
                    TID = [];
                    S = [];
                    EC = [];
                    Gain = [];
                    mIndex  = [];
                else
                    TID = fHist(frameNum).targetList.TID;
                    S = fHist(frameNum).targetList.S;
                    EC = fHist(frameNum).targetList.EC;
                    Gain = fHist(frameNum).targetList.G;
                    mIndex = fHist(frameNum).indexArray;
                end                
        end
                
        if nnz(isnan(S))
            disp('Error: S contains NaNs');
            continue;
        end
        if nnz(isnan(EC))
            disp('Error: EC contains NaNs');
            continue;
        end
        
        tNumC = length(TID);
        targetCountTotal = tNumC;
        targetCountInBox = zeros(scene.numberOfTargetBoxes,1);
 
        if(size(mIndex,1)) 
            mIndex = mIndex + 1;
        end
        
        point3D = [posAll; pointCloud(3,:)];
        % Plot previous frame's 3D points       
        if(size(point3D,2))   
            if isempty(hPlotPoints3D)
                hPlotPoints3D = plot3(gatingAx, point3D(1,:), point3D(2,:), point3D(3,:),'.k');
            else
                set(hPlotPoints3D, 'XData', point3D(1,:),'YData', point3D(2,:), 'ZData', point3D(3,:));
            end
        end     
        
        for n=1:tNumC
            
            tid = TID(n)+1;            
            if(tid > maxNumTracks)
                disp('Error: TID is wrong');
                continue;
            end

            if( (size(mIndex,2) > 0) && (size(mIndex,2) == size(point3D,2)) )
                tColor = colors(mod(tid,length(colors))+1);
                ind = (mIndex == tid);
                if nnz(ind)
                    if isempty(trackingHist(tid).hPlotAssociatedPoints)
                        trackingHist(tid).hPlotAssociatedPoints = plot3(gatingAx, point3D(1,ind), point3D(2,ind), point3D(3,ind),'o', 'color', tColor);
                    else
                        if ishandle(trackingHist(tid).hPlotAssociatedPoints)
                            set(trackingHist(tid).hPlotAssociatedPoints, 'XData', point3D(1,ind),'YData', point3D(2,ind), 'ZData', point3D(3,ind));
                        end
                    end
                end
            end
               
            g = Gain(n);
            centroid = computeH(1, S(:,n));
            ec = reshape(EC(:,n),3,3);
            if(nnz(ec)>1)
                [xU, yU, zU, vU] = gatePlot3(gatingAx, 1, centroid, ec);
                if isempty(trackingHist(tid).hMeshU)
                    trackingHist(tid).hMeshU = mesh(gatingAx, xU.*sin(yU),xU.*cos(yU), zU);
                    trackingHist(tid).hMeshU.EdgeColor = [0.5 0.5 0.5];
                    trackingHist(tid).hMeshU.FaceColor = 'none';
                else
                    set(trackingHist(tid).hMeshU, 'XData', xU.*sin(yU),'YData',xU.*cos(yU), 'ZData', zU);
                end
            end
            if(g ~= 0)
                %            [gLim, a, b, c] = gateCreateLimLocal(8, ec, centroid, [4,1,0]);
                [xG, yG, zG, vG] = gatePlot3(gatingAx, g, centroid, ec);
                if isempty(trackingHist(tid).hMeshG)
                    trackingHist(tid).hMeshG = mesh(gatingAx, xG.*sin(yG),xG.*cos(yG), zG);
                    trackingHist(tid).hMeshG.EdgeColor = colors(mod(tid,length(colors))+1);
                    trackingHist(tid).hMeshG.FaceColor = 'none';
                else
                    set(trackingHist(tid).hMeshG, 'XData', xG.*sin(yG),'YData',xG.*cos(yG), 'ZData', zG);
                end
            end
            
            if(activeTracks(tid) == 0)
                activeTracks(tid) = 1;
                trackingHist(tid).tid = TID(n);
                trackingHist(tid).allocationTime = targetFrameNum;
                trackingHist(tid).tick = 1;
                trackingHist(tid).posIndex = 1;
                trackingHist(tid).histIndex = 1;
                trackingHist(tid).sHat(1,:) = S(:,n);
                trackingHist(tid).pos(1,:) = S(1:2,n);
                trackingHist(tid).hPlotTrack = plot(trackingAx, S(1,n), S(2,n), '.-', 'color', colors(mod(tid,length(colors))+1));
                trackingHist(tid).hPlotCentroid = plot(trackingAx, S(1,n), S(2,n), 'o', 'color', colors(mod(tid,length(colors))+1));
            else
                activeTracks(tid) = 1;                
                trackingHist(tid).tick = trackingHist(tid).tick + 1;
                
                trackingHist(tid).histIndex = trackingHist(tid).histIndex + 1;
                if(trackingHist(tid).histIndex > 1000)
                    trackingHist(tid).histIndex = 1;
                end
                trackingHist(tid).sHat(trackingHist(tid).histIndex,:) = S(:,n);
                trackingHist(tid).ec(trackingHist(tid).histIndex,:) = EC(:,n);
                
                trackingHist(tid).posIndex = trackingHist(tid).posIndex + 1;
                if(trackingHist(tid).posIndex > 100)
                    trackingHist(tid).posIndex = 1;
                end
                trackingHist(tid).pos(trackingHist(tid).posIndex,:) = S(1:2,n);

                if(trackingHist(tid).tick > 100)
                    set(trackingHist(tid).hPlotTrack, 'XData', [trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,1); trackingHist(tid).pos(1:trackingHist(tid).posIndex,1)], ...
                        'YData',[trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,2); trackingHist(tid).pos(1:trackingHist(tid).posIndex,2)]);
                else
                    set(trackingHist(tid).hPlotTrack, 'XData', trackingHist(tid).pos(1:trackingHist(tid).posIndex,1), ...
                        'YData',trackingHist(tid).pos(1:trackingHist(tid).posIndex,2));
                end                
                set(trackingHist(tid).hPlotCentroid,'XData',S(1,n),'YData',S(2,n));
                plot(dopplerAx, S(4,n), S(2,n), 'o', 'color', colors(mod(tid,length(colors))+1));
                
                for nBoxes = 1:scene.numberOfTargetBoxes
                    if( (S(1,n) > scene.targetBox(nBoxes,1)) && (S(1,n) < (scene.targetBox(nBoxes,1) + scene.targetBox(nBoxes,3))) && ...
                            (S(2,n) > scene.targetBox(nBoxes,2)) && (S(2,n) < (scene.targetBox(nBoxes,2)+scene.targetBox(nBoxes,4))) )
                        targetCountInBox(nBoxes) = targetCountInBox(nBoxes) +1;
                    end                
                end
            end
        end
        
        iDelete = find(activeTracks == 2);
        for n=1:length(iDelete)
            ind = iDelete(n);
            delete(trackingHist(ind).hPlotTrack);
            delete(trackingHist(ind).hPlotCentroid);
            delete(trackingHist(ind).hMeshU);
            delete(trackingHist(ind).hMeshG);
            delete(trackingHist(ind).hPlotAssociatedPoints);
            trackingHist(ind).hMeshU = [];
            trackingHist(ind).hMeshG = [];
            activeTracks(ind) = 0;
        end
        
        iReady = (activeTracks == 1);
        activeTracks(iReady) = 2;
        
        string{1} = sprintf('Frame #: %d',targetFrameNum);
        string{2} = sprintf('Detection Points: %d',numOutputPoints);
        string{3} = sprintf('Target Count: %d',targetCountTotal);
        
        string{4} = 'In Box Count: [';
        for nBoxes = 1:scene.numberOfTargetBoxes-1
            string{4} = [string{4}, sprintf('%d, ', targetCountInBox(nBoxes))];
        end
        string{4} = [string{4}, sprintf('%d]', targetCountInBox(scene.numberOfTargetBoxes))];
        
        for n=1:length(hStatGlobal)
            set(hStatGlobal(n),'String',string{n});
        end
        
        for nBoxes = 1:scene.numberOfTargetBoxes
            if( (hTargetBoxHandle(nBoxes) ~= 0) && (ishandle(hTargetBoxHandle(nBoxes))))
                delete(hTargetBoxHandle(nBoxes));
            end
            if(targetCountInBox(nBoxes))
                hTargetBoxHandle(nBoxes) = rectangle(trackingAx, 'Position', scene.targetBox(nBoxes,:), 'EdgeColor','r', 'LineStyle', '-', 'LineWidth', 2);
            end
        end
        
        if captureVideo
            cdata = fGroup.cdata;
            dynammicFigures = [1, 2, 3, 8, 5];
            ns = 2;
            for n=1:length(dynammicFigures)
                f = getframe(hFigure(dynammicFigures(n)));
                indx = 49 + (1:size(f.cdata,1));
                indy = ns + (1:size(f.cdata,2));
                cdata(indx,indy,:) = f.cdata;
                ns = ns + size(f.cdata,2) + 7;
            end
            fSnapShot(frameNum).cdata = cdata;
        end
        
%{
        if hasFrame(videoObj)
            vidFrame = readFrame(videoObj);
        end
        image(vidFrame(:,20:700,:), 'Parent', groundTruthAx);
        hold(groundTruthAx, 'on');
        line('Parent', groundTruthAx, 'Xdata',[180 1035],'YData',[710, 710], 'Color','w', 'LineStyle', '-', 'LineWidth', 4);
        line('Parent', groundTruthAx, 'Xdata',[500 725],'YData',[1030, 405], 'Color','w', 'LineStyle', '--', 'LineWidth', 2);
        line('Parent', groundTruthAx, 'Xdata',[170 660],'YData',[1035, 407], 'Color','w', 'LineStyle', '--', 'LineWidth', 2);
        line('Parent', groundTruthAx, 'Xdata',[1 570],'YData',[850, 407], 'Color','w', 'LineStyle', '-', 'LineWidth', 2);
        line('Parent', groundTruthAx, 'Xdata',[1 630],'YData',[900, 390], 'Color','y', 'LineStyle', ':', 'LineWidth', 2);
        line('Parent', groundTruthAx, 'Xdata',[1 630],'YData',[1000, 407], 'Color','y', 'LineStyle', '--', 'LineWidth', 1);
        line('Parent', groundTruthAx, 'Xdata',[1 1035],'YData',[1035, 1035], 'Color','w', 'LineStyle', '-', 'LineWidth', 4);

        ind = find(gtruthFrame(:,1) == frameLocal);
        if ind 
            plot(groundTruthAx, gtruthFrame(ind,2), gtruthFrame(ind,3), 'o');
            plot(groundTruthAx, gtruthFrame(ind,2), gtruthFrame(ind,4), '.');
        end
        groundTruthAx.Visible = 'off';
        hold(groundTruthAx, 'off');
%}
        
%{        
%        F23.cdata = [F2(frameLocal).cdata F3(frameLocal).cdata];
        w = size(F2(frameLocal).cdata,1) - (size(F1(frameLocal).cdata,1) + size(F5.cdata,1) + size(F6.cdata,1)); 
        h = size(F1(frameLocal).cdata,2);
        fill = zeros(w,h,3, 'uint8') + 240;
        F1C(frameLocal).cdata = [F1(frameLocal).cdata; F5.cdata; F6.cdata; fill];
%        F(frameLocal).cdata = [F156.cdata F23.cdata];        
%}
        if(getappdata(hPbExit, 'exitKeyPressed') == 1)
            break;
        end
        pause(0.1);
    end
    matlabFileName = ['tmFrames_', num2str(fileNum, '%03d')];
    save(matlabFileName, 'frameWidth','frameHeight','fSnapShot','-v7.3');
    save('fhistTM.mat','fHist');
end
disp('Done');

%Display Chirp parameters in table on screen
function h = displayChirpParams(Params, Position)

    dat =  {'Start Frequency (Ghz)', Params.profileCfg.startFreq;...
            'Slope (MHz/us)', Params.profileCfg.freqSlopeConst;...   
            'Samples per chirp', Params.profileCfg.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Frame duration (ms)',  Params.frameCfg.framePeriodicity;...
            'Sampling rate (Msps)', Params.profileCfg.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.profileCfg.freqSlopeConst * Params.profileCfg.numAdcSamples /...
                               Params.profileCfg.digOutSampleRate;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Number of Rx (MIMO)', Params.dataPath.numRxAnt; ...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;};
    columnname =   {'Chirp Parameter (Units)      ', 'Value'};
    columnformat = {'char', 'numeric'};
    
    h = uitable('Units','normalized', ...
            'Position', Position, ...
            'Data', dat,... 
            'ColumnName', columnname,...
            'ColumnFormat', columnformat,...
            'ColumnWidth', 'auto',...
            'RowName',[]);
end

function [P] = parseCfg(cliCfg)
    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2double(C{3});
            P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                      bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
            P.dataPath.numTxElevAnt = 0;
            P.channelCfg.rxChannelEn = str2double(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'dataFmt')
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2double(C{3});
            P.profileCfg.idleTime =  str2double(C{4});
            P.profileCfg.rampEndTime = str2double(C{6});
            P.profileCfg.freqSlopeConst = str2double(C{9});
            P.profileCfg.numAdcSamples = str2double(C{11});
            P.profileCfg.digOutSampleRate = str2double(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2double(C{2});
            P.frameCfg.chirpEndIdx = str2double(C{3});
            P.frameCfg.numLoops = str2double(C{4});
            P.frameCfg.numFrames = str2double(C{5});
            P.frameCfg.framePeriodicity = str2double(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.detectedObjects = str2double(C{2});
            P.guiMonitor.logMagRange = str2double(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2double(C{4});
            P.guiMonitor.rangeDopplerHeatMap = str2double(C{5});
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
end

function stepPressFcn(hObject, ~)
    setappdata(hObject, 'stepKeyPressed', 1);
end
function exitPressFcn(hObject, ~)
    setappdata(hObject, 'exitKeyPressed', 1);
end

function clearCloudPressFcn(hObject, ~)
    sensor = getappdata(hObject, 'sensor');
    scene = getappdata(hObject, 'scene');
    cla;
    setCumCloudFigure(sensor, scene);
end

function clearDopplerPressFcn(hObject, ~)
    sensor = getappdata(hObject, 'sensor');
    cla;
    setDopplerMapFigure(sensor);
end

function setDopplerMapFigure(sensor)
    line('Xdata',[0 0],'YData',[0 80], 'Color','k', 'LineStyle', '--', 'LineWidth', 0.5);
    line('Xdata',[-sensor.maxRadialVelocity -sensor.maxRadialVelocity],'YData',[0 80], 'Color','r', 'LineStyle', '--', 'LineWidth', 0.5);
    line('Xdata',[sensor.maxRadialVelocity sensor.maxRadialVelocity],'YData',[0 80], 'Color','r', 'LineStyle', '--', 'LineWidth', 0.5);
    axis([-20 sensor.maxRadialVelocity+sensor.radialVelocityResolution 0 80]);
    xlabel('Doppler, m/s');
    ylabel('Y coordinate, m');
    grid on;
    hold on;
end

function setCumCloudFigure(sensor, scene)
    plot(sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
    plot([0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');
    xlabel('X coordinate, m');
    ylabel('Y coordinate, m');
    axis(scene.maxPos);
    grid on;
end

function config = readCfg(filename)
    config = cell(1,100);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    else
        fprintf('Opening configuration file %s ...\n', filename);
    end
    tline = fgetl(fid);
    k=1;
    while ischar(tline)
        config{k} = tline;
        tline = fgetl(fid);
        k = k + 1;
    end
    config = config(1:k-1);
    fclose(fid);
end

function [H] = computeH(~, s)
    posx = s(1); posy = s(2); velx = s(3); vely = s(4);
    range = sqrt(posx^2+posy^2);
    if posy == 0
        azimuth = pi/2;
    elseif posy > 0
        azimuth = atan(posx/posy);
    else
        azimuth = atan(posx/posy) + pi;
    end
    doppler = (posx*velx+posy*vely)/range;
    H = [range azimuth doppler]';
end

function [XX, YY, ZZ, v] = gatePlot3(~, G, C, A)
    %Extract the ellipsoid's axes lengths (a,b,c) and the rotation matrix (V) using singular value decomposition:
    [~,D,V] = svd(A/G);

    a = 1/sqrt(D(1,1));
    b = 1/sqrt(D(2,2));
    c = 1/sqrt(D(3,3));
    v = 4*pi*a*b*c/3;

    % generate ellipsoid at 0 origin
    [X,Y,Z] = ellipsoid(0,0,0,a,b,c);
    XX = zeros(size(X));
    YY = zeros(size(X));
    ZZ = zeros(size(X));
    for k = 1:length(X)
        for j = 1:length(X)
            point = [X(k,j) Y(k,j) Z(k,j)]';
            P = V * point;
            XX(k,j) = P(1)+C(1);
            YY(k,j) = P(2)+C(2);
            ZZ(k,j) = P(3)+C(3);
        end
    end
end

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
end
