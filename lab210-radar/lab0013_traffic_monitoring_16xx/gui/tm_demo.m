clear, clc, close all, delete(instrfind)

colors='bgrcm';
% ************************************************************************
hSetup = setup();
chirpParams = hSetup.params.dataPath;
hDataSerialPort = hSetup.DATACOM.hDataSerialPort;
hControlSerialPort = hSetup.UARTCOM.hControlSerialPort;
scene = hSetup.scene;
scene.numberOfTargetBoxes = scene.numberOfLanes;
scene.azimuthTilt =  hSetup.angle*pi/180;
scene.lineX = [scene.leftLineX scene.leftLineX+cumsum(scene.laneWidth)];
scene.numberOfTargetBoxes = scene.numberOfLanes;
scene.targetBox = [scene.lineX(1:end-1); repmat(scene.stopLineY(1), size(scene.laneWidth)); scene.laneWidth; repmat(scene.startLineY-scene.stopLineY(1),size(scene.laneWidth))]';
trackingAx = [];
gatingAx = [];
dopplerAx = [];
cumCloudAx = [];
global enableDopplerPlot;
global enableCumCloudPlot;
enableDopplerPlot = 0;
enableCumCloudPlot = 0;
% ************************************************************************
figHandle = figure('Name', 'Visualizer','tag','mainFigure');
clf(figHandle);
set(figHandle, 'WindowStyle','normal');
set(figHandle,'Name','Texas Instruments - People Counting','NumberTitle','off')    
set(figHandle,'currentchar',' ')         % set a dummy character
warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame
jframe=get(figHandle,'javaframe');
set(figHandle, 'MenuBar', 'none');
set(figHandle, 'Color', [0 0 0]);
pause(0.00001);
set(jframe,'Maximized',1); 
pause(0.00001);

figureTitles = {'Statistics', 'Point Cloud', 'Gating and Association', 'Doppler Map', 'Control'};% 'Cumulative Cloud'};
figureGroup = [1, 2, 3, 4, 5];
numFigures = size(figureTitles, 2);
hFigure = zeros(1,numFigures);

% Setup tab dimensions
hTabGroup(1) = uitabgroup(figHandle, 'Position', [0.0 0.5 0.19 0.5]);
hTabGroup(5) = uitabgroup(figHandle, 'Position', [0.0 0.0 0.19 0.5]);

hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.19 0.0 0.27 1]);
hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.46 0.0 0.27 1]);
hTabGroup(4) = uitabgroup(figHandle, 'Position', [0.73 0.0 0.27 1]);

% Populate tabs with plots
for iFig = 1:numFigures
    hFigure(iFig) = uitab(hTabGroup(figureGroup(iFig)), 'Title', figureTitles{iFig});
    if(strcmp(figureTitles{iFig},'Point Cloud') || strcmp(figureTitles{iFig},'Gating and Association'))
        %setup axes
        ax = axes('parent', hFigure(iFig));
        
        %copy axes from setup to figure
        fa = copyobj(hSetup.axes1.Children, ax);
        
        xlabel('X coordinate, m');
        ylabel('Y coordinate, m');
        axis equal;
        axis([hSetup.axes1.XLim hSetup.axes1.YLim]);
        grid on; hold on; grid minor;
        

        
        if(strcmp(figureTitles{iFig},'Point Cloud'))
            trackingAx = ax;
        end
        if(strcmp(figureTitles{iFig},'Gating and Association'))
            gatingAx = ax; 
%             for nBoxes = 1:scene.numberOfTargetBoxes
%                  hTargetBoxHandle(nBoxes) = rectangle(ax, 'Position', scene.targetBox(nBoxes,:), 'EdgeColor','r', 'LineStyle', '-', 'LineWidth', 2);
%             end
        end
    end
    
%     if(strcmp(figureTitles{iFig},'Chirp Configuration'))
%         tablePosition = [0.1 0.1 0.8 0.8];
%         h = displayChirpParams(Params, tablePosition);
%         h.InnerPosition = [h.InnerPosition(1:2) h.Extent(3:4)];        
%     end
    
    if(strcmp(figureTitles{iFig},'Statistics'))
        ax = axes('parent', hFigure(iFig));
        hStatGlobal(1) = text(0, 0.9, 'Frame # 0', 'FontSize',12);
        hStatGlobal(2) = text(0, 0.7, 'Detection Points: 0','FontSize',12);
        hStatGlobal(3) = text(0, 0.5, 'Target Count:  0','FontSize',12);
        hStatGlobal(4) = text(0, 0.3, 'In Box Count:  0','FontSize',12);
        hStatGlobal(5) = text(0, 0.1, 'Bytes Available:  0/0','FontSize',12);
        axis off;
    end
    
    if(strcmp(figureTitles{iFig},'Control'))
        cFig = iFig;
        hRbPause = uicontrol(hFigure(cFig),'Style','radio','String','Pause','FontSize', 12,...
            'Units', 'normalized', 'Position',[0.1 0.1 0.3 0.1],'Value',0);
        hPbExit = uicontrol(hFigure(cFig),'Style', 'pushbutton', 'String', 'Exit','FontSize', 12,...
            'Units', 'normalized','Position', [0.6 0.1 0.3 0.1],'Callback', @exitPressFcn);
        hCbEnableDoppler = uicontrol(hFigure(cFig),'Style', 'checkbox', 'String', 'Enable Doppler Plot','Callback', @cbDoppler,...
            'Units', 'normalized','Position', [0.1 0.8 0.5 0.1],'FontSize', 12);
        setappdata(hPbExit, 'exitKeyPressed', 0);
    end
    
    if(strcmp(figureTitles{iFig},'Doppler Map'))
        ax = axes('parent', hFigure(iFig));
        line('Xdata',[0 0],'YData',hSetup.plotBound.Y, 'Color','k', 'LineStyle', '--', 'LineWidth', 0.5);
        line('Xdata',[-chirpParams.maxVelocity -chirpParams.maxVelocity],'YData',hSetup.plotBound.Y, 'Color','r', 'LineStyle', '--', 'LineWidth', 0.5);
        line('Xdata',[chirpParams.maxVelocity chirpParams.maxVelocity],'YData',hSetup.plotBound.Y, 'Color','r', 'LineStyle', '--', 'LineWidth', 0.5);
        axis([-20 chirpParams.maxVelocity+chirpParams.dopplerResolutionMps hSetup.plotBound.Y]);
        xlabel('Doppler, m/s');
        ylabel('Y coordinate, m');
        grid on;
        hold on;
        dopplerAx = gca;
    end
    if(strcmp(figureTitles{iFig},'Cum Cloud'))
        ax = axes('parent', hFigure(iFig));
              %copy axes from setup to figure
        fa = copyobj(hSetup.axes1.Children, ax);
        
        xlabel('X coordinate, m');
        ylabel('Y coordinate, m');
        % axis equal;
        axis([hSetup.axes1.XLim hSetup.axes1.YLim]);
        grid on; hold on;
        cumCloudAx = gca;
    end 
    
    %% Webcam setup - TO DO: Implement
%     if(strcmp(figureTitles{iFig},'Ground Truth'))
%         if(~(hSetup.camIndex == -1))
%             enableWebcam = 1;
%             cam = webcam(hSetup.camIndex);
%             resList = cam.AvailableResolution;
%             cam.Resolution = resList{getWidestFOV(resList)};
% 
%             axWebcam = axes('Parent', hFigure(iFig));
%             hImage = image(axWebcam, snapshot(cam));
%             axis(axWebcam, 'manual','off')
% 
% 
%             % Set up the push buttons
%             uicontrol('String', 'Play',...
%                 'Callback', 'preview(cam, hImage)',...
%                 'Units','normalized',...
%                 'Position',[0 0 0.15 .07]);
%             uicontrol('String', 'Pause',...
%                 'Callback', 'closePreview(cam)',...
%                 'Units','normalized',...
%                 'Position',[.17 0 .15 .07]);
% 
% %             uicontrol('String', 'Close',...
% %                 'Callback', 'delete(hWebcamFigure)',...
% %                 'Units','normalized',...
% %                 'Position',[0.34 0 .15 .07]);
% 
% 
% %             axWebcam = axes('Parent', hWebcamFigure);
% %             hImage = image(axWebcam, snapshot(cam));
% %             axis(axWebcam, 'manual','off')
% 
% 
%             res = cam.Resolution;
%             ss = strsplit(res,'x');
%             imWidth = str2num(ss{1});
%             imHeight = str2num(ss{2});
%             hImage = image( zeros(imHeight, imWidth, 3) );
%             % Set up the update preview window function.
% %             setappdata(hImage,'UpdatePreviewWindowFcn',@mypreview_fcn);
% 
% 
% 
%             % Specify the size of the axes that contains the image object
%             % so that it displays the image at the right resolution and
%             % centers it in the figure window.
%             figSize = get(hWebcamFigure,'Position');
%             figWidth = figSize(3);
%             figHeight = figSize(4);
%             gca.unit = 'pixels';
%             gca.position = [ ((figWidth - imWidth)/2)... 
%                            ((figHeight - imHeight)/2)...
%                            imWidth imHeight ];
% 
% 
%             hCam = preview(cam, hImage);
%             pause(0.5); %allow webcam to load
%         else
%             enableWebcam = 0;
%         end
%     end
end
close(hSetup.figure1);

trackerRun = 'Target';
fileLoop = 1;
fileFrameSize = 1000;


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
    trackerConfig.maxAcceleration = [0 2];
    trackerConfig.deltaT = sensor.framePeriod/1000; %in sec
    trackerConfig.initialRadialVelocity = 0;
    trackerConfig.ax = [];
    
    boundaryBoxes = [scene.boundaryBox(:,1), scene.boundaryBox(:,1)+scene.boundaryBox(:,3), scene.boundaryBox(n,2), scene.boundaryBox(n,2)+scene.boundaryBox(n,4)];
    staticBoxes = [scene.staticBox(:,1), scene.staticBox(:,1)+scene.staticBox(n,3), scene.staticBox(n,2), scene.staticBox(n,2)+scene.staticBox(n,4)];

    trackerConfig.advParams.scenery = [scene.numberOfBoundaryBoxes, boundaryBoxes scene.numberOfStaticBoxes, staticBoxes];
    trackerConfig.advParams.allocation = [100, 100, 1, 3, 2.8, 5*sensor.radialVelocityResolution]; % Total SNR, min Velocity, min Points, distance, velocity separation
    trackerConfig.advParams.gating = [16, 12,6,0]; % Volume, Length Width and Velocity Limits
    trackerConfig.advParams.thresholds = [3, 3, 5, 1000, 5]; % det2act, det2free, act2free, stat2free, exit2free
    trackerConfig.advParams.variations = [4/sqrt(12), 1.5/sqrt(12), 1]; % Height, Width, Doppler std
end

if(strcmp(trackerRun,'PCMex'))
    trackerConfig.verbose = 3;
    hRadarTrackingC = gtrack_create_mex(trackerConfig);
end
if(strcmp(trackerRun,'PCMatlab'))
    trackerConfig.verbose = 5;
    hRadarTrackingM = trackModule('trackerConfig', trackerConfig);
end

%Configure data UART port with input buffer to hold 100+ frames 


syncPatternUINT64 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint64');
syncPatternUINT8 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint8');

frameHeaderStructType = struct(...
    'sync',             {'uint64', 8}, ... % See syncPatternUINT64 below
    'version',          {'uint32', 4}, ...
    'platform',         {'uint32', 4}, ...
    'timestamp',        {'uint32', 4}, ... % 600MHz clocks
    'packetLength',     {'uint32', 4}, ... % In bytes, including header
    'frameNumber',      {'uint32', 4}, ... % Starting from 1
    'subframeNumber',   {'uint32', 4}, ...
    'chirpMargin',      {'uint32', 4}, ... % Chirp Processing margin, in ms
    'frameMargin',      {'uint32', 4}, ... % Frame Processing margin, in ms
    'uartSentTime' ,    {'uint32', 4}, ... % Time spent to send data, in ms
    'trackProcessTime', {'uint32', 4}, ... % Tracking Processing time, in ms
    'numTLVs' ,         {'uint16', 2}, ... % Number of TLVs in thins frame
    'checksum',         {'uint16', 2});    % Header checksum

tlvHeaderStruct = struct(...
    'type',             {'uint32', 4}, ... % TLV object Type
    'length',           {'uint32', 4});    % TLV object Length, in bytes, including TLV header 

% Point Cloud TLV object consists of an array of points. 
% Each point has a structure defined below
pointStruct = struct(...
    'range',            {'float', 4}, ... % Range, in m
    'angle',            {'float', 4}, ... % Angel, in rad
    'doppler',          {'float', 4}, ... % Doplper, in m/s
    'snr',              {'float', 4});    % SNR, ratio
% Target List TLV object consists of an array of targets. 
% Each target has a structure define below
targetStruct = struct(...
    'tid',              {'uint32', 4}, ... % Track ID
    'posX',             {'float', 4}, ... % Target position in X dimension, m
    'posY',             {'float', 4}, ... % Target position in Y dimension, m
    'velX',             {'float', 4}, ... % Target velocity in X dimension, m/s
    'velY',             {'float', 4}, ... % Target velocity in Y dimension, m/s
    'accX',             {'float', 4}, ... % Target acceleration in X dimension, m/s2
    'accY',             {'float', 4}, ... % Target acceleration in Y dimension, m/s
    'EC',               {'float', 9*4}, ... % Tracking error covariance matrix, [3x3], in range/angle/doppler coordinates
    'G',                {'float', 4});    % Gating function gain

frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType);
tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct);
pointLengthInBytes = lengthFromStruct(pointStruct);
targetLengthInBytes = lengthFromStruct(targetStruct);
indexLengthInBytes = 1;

exitRequest = 0;
lostSync = 0;
gotHeader = 0;
outOfSyncBytes = 0;
runningSlow = 0;
maxBytesAvailable = 0;
point3D = [];

frameStatStruct = struct('targetFrameNum', [], 'bytes', [], 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, 'start', 0, 'benchmarks', [], 'done', 0, ...
    'pointCloud', [], 'targetList', [], 'indexArray', []);
fHist = repmat(frameStatStruct, 1, fileFrameSize);

skipProcessing = 0;
frameNum = 1;
frameNumLogged = 1;
fprintf('------------------\n');

while(1)
    while(lostSync == 0)

        frameStart = tic;
        fHist(frameNum).timestamp = frameStart;
        bytesAvailable = get(hDataSerialPort,'BytesAvailable');
        if(bytesAvailable > maxBytesAvailable)
            maxBytesAvailable = bytesAvailable;
        end
        fHist(frameNum).bytesAvailable = bytesAvailable;
        if(gotHeader == 0)
            %Read the header first
            [rxHeader, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes, 'uint8');
        end
        fHist(frameNum).start = 1000*toc(frameStart);
        
        magicBytes = typecast(uint8(rxHeader(1:8)), 'uint64');
        if(magicBytes ~= syncPatternUINT64)
            reason = 'No SYNC pattern';
            lostSync = 1;
            break;
        end
        if(byteCount ~= frameHeaderLengthInBytes)
            reason = 'Header Size is wrong';
            lostSync = 1;
            break;
        end        
        if(validateChecksum(rxHeader) ~= 0)
            reason = 'Header Checksum is wrong';
            lostSync = 1;
            break; 
        end
        
        frameHeader = readToStruct(frameHeaderStructType, rxHeader);
        
        if(gotHeader == 1)
            if(frameHeader.frameNumber > targetFrameNum)
                targetFrameNum = frameHeader.frameNumber;
                disp(['Found sync at frame ',num2str(targetFrameNum),'(',num2str(frameNum),'), after ', num2str(1000*toc(lostSyncTime),3), 'ms']);
                gotHeader = 0;
            else
                reason = 'Old Frame';
                gotHeader = 0;
                lostSync = 1;
                break;
            end
        end
        
        % We have a valid header
        targetFrameNum = frameHeader.frameNumber;
        fHist(frameNum).targetFrameNum = targetFrameNum;
        fHist(frameNum).header = frameHeader;
        
        dataLength = frameHeader.packetLength - frameHeaderLengthInBytes;
        
        fHist(frameNum).bytes = dataLength; 
        numInputPoints = 0;
        numTargets = 0;
        mIndex = [];

        if(dataLength > 0)
            %Read all packet
            [rxData, byteCount] = fread(hDataSerialPort, double(dataLength), 'uint8');
            if(byteCount ~= double(dataLength))
                reason = 'Data Size is wrong'; 
                lostSync = 1;
                break;  
            end
            offset = 0;
    
            fHist(frameNum).benchmarks(1) = 1000*toc(frameStart);

            % TLV Parsing
            for nTlv = 1:frameHeader.numTLVs
                tlvType = typecast(uint8(rxData(offset+1:offset+4)), 'uint32');
                tlvLength = typecast(uint8(rxData(offset+5:offset+8)), 'uint32');
                if(tlvLength + offset > dataLength)
                    reason = 'TLV Size is wrong';
                    lostSync = 1;
                    break;                    
                end
                offset = offset + tlvHeaderLengthInBytes;
                valueLength = tlvLength - tlvHeaderLengthInBytes;
                switch(tlvType)
                    case 6
                        % Point Cloud TLV
                        numInputPoints = valueLength/pointLengthInBytes;
                        if(numInputPoints > 0)                        
                            % Get Point Cloud from the sensor
                            p = typecast(uint8(rxData(offset+1: offset+valueLength)),'single');

                            pointCloud = reshape(p,4, numInputPoints);    
%                            pointCloud(2,:) = pointCloud(2,:)*pi/180;

                            posAll = [pointCloud(1,:).*sin(pointCloud(2,:)); pointCloud(1,:).*cos(pointCloud(2,:))];
                            snrAll = pointCloud(4,:);
%{
                            % Remove out of Range, Behind the Walls, out of FOV points
                            inRangeInd = (pointCloud(1,:) > 1) & (pointCloud(1,:) < 6) & ...
                                (pointCloud(2,:) > -50*pi/180) &  (pointCloud(2,:) < 50*pi/180) & ...
                                (posAll(1,:) > scene.areaBox(1)) & (posAll(1,:) < (scene.areaBox(1) + scene.areaBox(3))) & ...
                                (posAll(2,:) > scene.areaBox(2)) & (posAll(2,:) < (scene.areaBox(2) + scene.areaBox(4)));
                            pointCloudInRange = pointCloud(:,inRangeInd);
                            posInRange = posAll(:,inRangeInd);
%}                            
%{
                            % Clutter removal
                            staticInd = (pointCloud(3,:) == 0);        
                            clutterInd = ismember(pointCloud(1:2,:)', clutterPoints', 'rows');
                            clutterInd = clutterInd' & staticInd;
                            clutterPoints = pointCloud(1:2,staticInd);
                            pointCloud = pointCloud(1:3,~clutterInd);
%}
                            numOutputPoints = size(pointCloud,2);                          
                        end                        
                        offset = offset + valueLength;
                                            
                    case 7
                        % Target List TLV
                        numTargets = valueLength/targetLengthInBytes;                        
                        TID = zeros(1,numTargets);
                        S = zeros(6, numTargets);
                        EC = zeros(9, numTargets);
                        G = zeros(1,numTargets);                        
                        for n=1:numTargets
                            TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');      %1x4=4bytes
                            S(:,n)  = typecast(uint8(rxData(offset+5:offset+28)),'single');     %6x4=24bytes
                            EC(:,n) = typecast(uint8(rxData(offset+29:offset+64)),'single');    %9x4=36bytes
                            G(n)    = typecast(uint8(rxData(offset+65:offset+68)),'single');    %1x4=4bytes
                            offset = offset + 68;
                        end
                        
                    case 8
                        % Target Index TLV
                        numIndices = valueLength/indexLengthInBytes;
                        mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                        offset = offset + valueLength;
                end
            end
        end
       
        if(numInputPoints == 0)
            numOutputPoints = 0;
            pointCloud = single(zeros(4,0));
            posAll = [];
            posInRange = [];  
        end
        if(numTargets == 0)
            TID = [];
            S = [];
            EC = [];
            G = [];
        end
        
        fHist(frameNum).numInputPoints = numInputPoints;
        fHist(frameNum).numOutputPoints = numOutputPoints;    
        fHist(frameNum).numTargets = numTargets;
        fHist(frameNum).pointCloud = pointCloud;
        fHist(frameNum).targetList.numTargets = numTargets;
        fHist(frameNum).targetList.TID = TID;
        fHist(frameNum).targetList.S = S;
        fHist(frameNum).targetList.EC = EC;
        fHist(frameNum).targetList.G = G;
        fHist(frameNum).indexArray = mIndex;
       
        % Plot pointCloud
        fHist(frameNum).benchmarks(2) = 1000*toc(frameStart);
   
        if(get(hRbPause, 'Value') == 1)
            pause(0.01);
            continue;
        end
        
        % Delete previous points
        if(ishandle(hPlotCloudHandleAll))
            delete(hPlotCloudHandleAll);
        end
        if(ishandle(hPlotCloudHandleOutRange))
            delete(hPlotCloudHandleOutRange);
        end
        if(ishandle(hPlotCloudHandleClutter))
            delete(hPlotCloudHandleClutter);
        end
        if(ishandle(hPlotCloudHandleStatic))
            delete(hPlotCloudHandleStatic);
        end
        if(ishandle(hPlotCloudHandleDynamic))
            delete(hPlotCloudHandleDynamic);
        end

        if(size(posAll,2))
            % Plot all points
            if(snrAll*10 > 0)
                hPlotCloudHandleAll = scatter(trackingAx, posAll(1,:), posAll(2,:),'.k','SizeData',snrAll*10);
            else
                reason = 'SNR value is wrong';
                lostSync = 1;
                break;                
            end
            if (enableDopplerPlot && ~isempty(dopplerAx))
                plot(dopplerAx, pointCloud(3,:), posAll(2,:), '.k');
            end
            if (enableCumCloudPlot && ~isempty(cumCloudAx))
                plot3(cumCloudAx, posAll(1,:), posAll(2,:), pointCloud(4,:),'.b');
            end
        end
        
%{        
        if(size(posInRange,2))
            % Cross out Clutter
            hPlotCloudHandleClutter = plot(trackingAx, posInRange(1,clutterInd), posInRange(2,clutterInd), 'xk');
            % Indicate Static
            hPlotCloudHandleStatic = plot(trackingAx, posInRange(1,staticInd & ~clutterInd), posInRange(2,staticInd & ~clutterInd), 'ok');
            % Indicate Dynamic
            hPlotCloudHandleDynamic = plot(trackingAx, posInRange(1,~staticInd), posInRange(2,~staticInd), 'ob');
        end
%}        
        fHist(frameNum).benchmarks(3) = 1000*toc(frameStart);

        switch trackerRun
            case 'PCMex'        
                [TID, S, EC, G, mIndex] = gtrack_step_mex(hRadarTrackingC, pointCloud, [], numOutputPoints);
                    
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
                    G(n) = track(n).g;
                end
                mIndex = mIndex -1;

            case 'Target'
                if(numTargets == 0)
                    TID = zeros(1,0);
                    S = zeros(6,0);
                    EC = zeros(9,0);
                    G = zeros(1,0);
                end
        end
        
        fHist(frameNum).benchmarks(4) = 1000*toc(frameStart);
        
        if nnz(isnan(S))
            reason = 'Error: S contains NaNs';
            lostSync = 1;
            break;
        end
        if nnz(isnan(EC))
            reason = 'Error: EC contains NaNs';
            lostSync = 1;
            break;
        end
        
        tNumC = length(TID);
        targetCountTotal = tNumC;
        targetCountInBox = zeros(scene.numberOfTargetBoxes,1);
 
        if(size(mIndex,1)) 
            mIndex = mIndex + 1;
        end
        
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
                reason = 'Error: TID is wrong';
                lostSync = 1;
                break;
            end

            if( (size(mIndex,1) > 0) && (size(mIndex,1) == size(point3D,2)) )
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
               
            g = G(n);
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
            if(~isnan(g) && (g ~= 0))
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
                if (enableDopplerPlot &&~isempty(dopplerAx))            
                    plot(dopplerAx, S(4,n), S(2,n), 'o', 'color', colors(mod(tid,length(colors))+1));
                end
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
        
        fHist(frameNum).done = 1000*toc(frameStart);
        
        string{1} = sprintf('Frame #: %d',targetFrameNum);
        string{2} = sprintf('Detection Points: %d',numOutputPoints);
        string{3} = sprintf('Target Count: %d',targetCountTotal);
        
        string{4} = 'In Box Count: [';
        for nBoxes = 1:scene.numberOfTargetBoxes-1
            string{4} = [string{4}, sprintf('%d, ', targetCountInBox(nBoxes))];
        end
        string{4} = [string{4}, sprintf('%d]', targetCountInBox(scene.numberOfTargetBoxes))];
        string{5} = sprintf('Bytes Available: %d', bytesAvailable);
        
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
        
        if(getappdata(hPbExit, 'exitKeyPressed') == 1)
            matlabFileName = [hSetup.savePath '\fHistRT_', num2str(fileLoop, '%04d'), '.mat'];
            fHist = fHist(1:frameNum);
            save(matlabFileName,'fHist');        
            disp(['Saving data in ', matlabFileName, ' ...']);
%             % Close and delete handles before exiting
%             % close(1); % close figure
%                 %load Cfg Params
%             mmwDemoCliPrompt = char('mmwDemo:/>');
%             command = 'sensorStop';
%             fprintf(hControlSerialPort, command);
%             fprintf('%s\n', command);
%             echo = fgetl(hControlSerialPort); % Get an echo of a command
%             done = fgetl(hControlSerialPort); % Get "Done" 
%             prompt = fread(hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back 
%    
            fclose(hDataSerialPort); %close com port
            delete(hDataSerialPort);
            fclose(hControlSerialPort); %close com port
            delete(hControlSerialPort);
            return;
        end
        
        frameNum = frameNum + 1;
        
        if(frameNum > fileFrameSize)
            matlabFileName = [hSetup.savePath '\fHistRT_', num2str(fileLoop, '%04d'), '.mat'];
            save(matlabFileName,'fHist');        
            disp(['Saving data in ', matlabFileName, ' ...']);
            frameNum = 1;
            fileLoop = fileLoop + 1;
        end
                
        point3D = [posAll; pointCloud(3,:)];
        
        if(bytesAvailable > 32000)
            runningSlow  = 1;
        elseif(bytesAvailable < 1000)
            runningSlow = 0;
        end
        
        if(runningSlow)
            % Don't pause, we are slow
        else
            pause(0.01);
        end
    end
    
    lostSyncTime = tic;
    bytesAvailable = get(hDataSerialPort,'BytesAvailable');
    disp(['Lost sync at frame ', num2str(targetFrameNum),'(', num2str(frameNum), '), Reason: ', reason, ', ', num2str(bytesAvailable), ' bytes in Rx buffer']);
%{
    % To catch up, we read and discard all uart data
    bytesAvailable = get(hDataSerialPort,'BytesAvailable');
    disp(bytesAvailable);
    [rxDataDebug, byteCountDebug] = fread(hDataSerialPort, bytesAvailable, 'uint8');
%}    
    while(lostSync)
        for n=1:8
            [rxByte, byteCount] = fread(hDataSerialPort, 1, 'uint8');
            if(rxByte ~= syncPatternUINT8(n))
                outOfSyncBytes = outOfSyncBytes + 1;
                break;
            end
        end
        if(n == 8)
            lostSync = 0;
            frameNum = frameNum + 1;
            if(frameNum > 10000)
                frameNum = 1;
            end
            
            [header, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes - 8, 'uint8');
            rxHeader = [syncPatternUINT8'; header];
            byteCount = byteCount + 8;
            gotHeader = 1;
        end
    end
end

function [] = dispError()
    disp('Serial Port Error!');
end

function exitPressFcn(hObject, ~)
    setappdata(hObject, 'exitKeyPressed', 1);
end

function [sphandle] = configureDataSport(comPortNum, bufferSize)
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);
end

function [sphandle] = configureControlPort(comPortNum)
    %if ~isempty(instrfind('Type','serial'))
    %    disp('Serial port(s) already open. Re-initializing...');
    %    delete(instrfind('Type','serial'));  % delete open serial ports.
    %end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')    
    set(sphandle,'Terminator','LF')        
    fopen(sphandle);
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

function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end
end

function [R] = readToStruct(S, ByteArray)
    fieldName = fieldnames(S);
    offset = 0;
    for n = 1:numel(fieldName)
        [fieldType, fieldLength] = S.(fieldName{n});
        R.(fieldName{n}) = typecast(uint8(ByteArray(offset+1:offset+fieldLength)), fieldType);
        offset = offset + fieldLength;
    end
end
function CS = validateChecksum(header)
    h = typecast(uint8(header),'uint16');
    a = uint32(sum(h));
    b = uint16(sum(typecast(a,'uint16')));
    CS = uint16(bitcmp(b));
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

function [resInd] = getWidestFOV(resList)
maxR = 1;
resInd = 1;
    for i=1:length(resList)
        ss = strsplit(resList{i},'x');
        imWidth = str2num(ss{1});
        imHeight = str2num(ss{2});
        r = imWidth/imHeight;
        if (r>maxR)
            maxR = r;
            resInd = i;
        end
    end
end

function cbDoppler(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
global enableDopplerPlot
enableDopplerPlot = (get(hObject,'Value'));
end