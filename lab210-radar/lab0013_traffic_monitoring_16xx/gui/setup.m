function varargout = setup(varargin)
% SETUP MATLAB code for setup.fig
%      SETUP, by itself, creates a new SETUP or raises the existing
%      singleton*.
%
%      H = SETUP returns the handle to a new SETUP or the handle to
%      the existing singleton*.
%
%      SETUP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETUP.M with the given input arguments.
%
%      SETUP('Property','Value',...) creates a new SETUP or raises
%      the existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setup_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setup_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setup

% Last Modified by GUIDE v2.5 14-Apr-2018 22:19:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @setup_OpeningFcn, ...
                   'gui_OutputFcn',  @setup_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before setup is made visible.
function setup_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to setup (see VARARGIN)

% Choose default command line output for setup
handles.output = hObject;
%handles.camIndex = -1;
handles.hDataSerialPort = [];
handles.hControlSerialPort = [];
handles.scene = struct('numberOfLanes', 3, 'laneWidth', [3.5,3.5,3.5], 'leftLineX', 1, 'stopLineY', [20 20 20 20], 'startLineY', 50);
handles.plotBound = struct('X', [-5,15], 'Y', [-5,80]);
handles.angle = 0;
handles.box = struct('numBoundary', 1, 'boundaryBox', [-2 15, 10.0, 75.0; 0, 0, 0, 0], 'numStatic', 1, 'staticBox',[1.25 11.0, 20.0, 50.0; 0, 0, 0, 0]);  
handles.cfg = struct('filename', 'mmw_tm_demo_default.cfg', 'loaded', 0);
handles.savePath = pwd;

% Update handles structure
guidata(hObject, handles);

initialize_gui(hObject, handles, false);
hsavePath = findobj('Tag', 'textSavePath');
hsavePath.String = ['Save path: ' handles.savePath];
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = setup_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles;



% --- Executes on button press in btnStart.
function btnStart_Callback(hObject, eventdata, handles)
% hObject    handle to btnStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% check ports connected
if(length(instrfind('Type','serial', 'Status','open'))>=2 && ~isempty(handles.UARTCOM.hControlSerialPort) && ~isempty(handles.DATACOM.hDataSerialPort))
    
    %check for overrides
    hOverride = findobj('Tag','checkboxOverride');
    sp = [];
    ap = [];
    stp = [];
    
    if(hOverride.Value)
        % sceneryParam
        bb = reshape(handles.box.boundaryBox', 1, []);
        sb = reshape(handles.box.staticBox', 1, []);
        temp = [sprintf(' %d',handles.box.numBoundary) sprintf(' %0.1f',bb) sprintf(' %d',handles.box.numStatic) sprintf(' %0.1f',sb)];
        sp = ['sceneryParam' temp];
        % allocationParam
        ha = findobj('Tag','uitableAllocation');
        ap = [ha.Data{1} sprintf(' %s', ha.Data{2:end})];  
        % stateTransitionParam
        hstp = findobj('Tag','uitableState');
        stp = [hstp.Data{1} sprintf(' %s', hstp.Data{2:end})]; 
    end
    
    disp(sp);
    disp(ap);
    disp(stp);
    
    %error check for azimuth tilt
    angleError = 0;
    if(isfield(handles.params,'trackingCfg'))
        t = handles.params.trackingCfg;
        ang = t{end};
        tc = [];

        if((90-str2num(ang)) ~= handles.angle) % check if match
            new_ang = num2str(90-handles.angle);
            temp = sprintf('%s ',t{1:end-1});
            temp = [temp new_ang]; 
            tc = temp;
            angleError = 1;
            fprintf('Warning: trackingCfg specifies %d.\n', (str2num(ang)-90)*pi/180 );
            fprintf('GUI specifies %d. %s will be used for azimuth tilt.\n',handles.angle, new_ang);
        end
    end

    %load Cfg Params
    mmwDemoCliPrompt = char('mmwDemo:/>');

    %Send CLI configuration to IWR16xx
    fprintf('Sending configuration from %s file to IWR16xx ...\n', handles.cfg.filename);
    
    for k=1:length(handles.cfg.cliCfg)
        command = handles.cfg.cliCfg{k};
        c1 = strsplit(command);
        if(hOverride.Value)
            if(strcmp(c1{1},'sceneryParam'))
                command = sp;
            elseif(strcmp(c1{1},'allocationParam'))
                command = ap;
            elseif(strcmp(c1{1},'stateParam'))
                command = stp;
            end
        end 
        
        if(angleError)
            if(strcmp(c1{1},'trackingCfg'))
                command = tc;
            end
        end
        
        fprintf(handles.UARTCOM.hControlSerialPort, command);
        fprintf('%s\n', command);
        echo = fgetl(handles.UARTCOM.hControlSerialPort); % Get an echo of a command
        done = fgetl(handles.UARTCOM.hControlSerialPort); % Get "Done" 
        prompt = fread(handles.UARTCOM.hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back 
    end
    %fclose(handles.UARTCOM.hControlSerialPort);
    %delete(hControlSerialPort);

    setup_OutputFcn(hObject,eventdata,guidata(hObject));
    uiresume(gcbf);
else
    warndlg('Error: Can not start COM ports not connected. Please select and connect.');
end
    


% --- Executes on button press in btnCancel.
function btnCancel_Callback(hObject, eventdata, handles)
% hObject    handle to btnCancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(instrfind('Type','serial', 'Status','open'))
initialize_gui(gcbf, handles, true);
close(gcbf)




% --------------------------------------------------------------------
function initialize_gui(fig_handle, handles, isreset)


% Update handles structure
guidata(handles.figure1, handles);





% --- Executes on button press in pushbuttonBrowse.
function pushbuttonBrowse_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonBrowse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Select Chirp Config File
[filename, pathname] = uigetfile('*.cfg','*.*');
configurationFileName = [pathname filename];
handles.cfg.filename = configurationFileName;

if (filename ~= 0)
    % Read Chirp Configuration file
    cliCfg = readCfg(handles.cfg.filename);
    [Params cliCfg] = parseCfg(cliCfg); 
    
    % Display chirp table
    hTable = findobj('Tag', 'uitableChirp');
    hTable = displayChirpParams(Params, hTable);
    hGtrackTable = findobj('Tag', 'uitableGtrackParams');
    
    % Update handles
    handles.cfg.cliCfg = cliCfg;
    handles.params = Params;
    guidata(hObject,handles)
    
    % Display gtrack params
    % check if params set in CLI commands
    if(isfield(Params, 'gtrackAdvanced'))
        if(isfield(Params.gtrackAdvanced,'sceneryParam'))
            temp = Params.gtrackAdvanced.sceneryParam;
            temp =[reshape(temp(3:10),4,2)'; reshape(temp(12:19),4,2)'];
            temp = cellfun(@str2num,temp);
            hTable = findobj('Tag', 'uitableScenery');
            displayGtrackParams(temp, hTable);
            data = hTable.Data;
            handles.box.boundaryBox = data(1:2,:);%cellfun(@str2num,data(1:2,:));
            handles.box.staticBox = data(3:4,:); %cellfun(@str2num,data(3:4,:));
            handles.box.numBoundary = sum(any(handles.box.boundaryBox,2));
            handles.box.numStatic = sum(any(handles.box.staticBox,2));
            guidata(hObject,handles)
            drawRadarRoom(handles);
        end
        
        if(isfield(Params.gtrackAdvanced,'allocationParam'))
            displayGtrackParams(Params.gtrackAdvanced.allocationParam, findobj('Tag', 'uitableAllocation'));
        end
        
        if(isfield(Params.gtrackAdvanced,'stateParam'))
            displayGtrackParams(Params.gtrackAdvanced.stateParam, findobj('Tag', 'uitableState'));
        end
    end

    
    % Update scenePreview
    drawRadarRoom(handles);
end
guidata(hObject,handles)


% --- Executes on button press in btnConnect.
function btnConnect_Callback(hObject, eventdata, handles)
% hObject    handle to btnConnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

hEditUART = findobj('Tag','editUART');
hEditData = findobj('Tag','editData');

handles.UARTCOM.num = str2num(hEditUART.String);
handles.DATACOM.num = str2num(hEditData.String); 

% Clear ports
if ~isempty(instrfind('Type','serial'))
    disp('Serial port(s) already open. Re-initializing...');
    delete(instrfind('Type','serial'));  % delete open serial ports.
end

% Configure data port
hDataSerialPort = configureDataPort(handles.DATACOM.num, 65536);
% Configure UART port
hControlSerialPort = configureControlPort(handles.UARTCOM.num);

% Update COM status msg
hCOMStatus = findobj('Tag', 'textCOMStatus');
update = 'COM STATUS: Ports connected';
set(hCOMStatus,'String', update);

%Update Handles
handles.DATACOM.hDataSerialPort = hDataSerialPort;
handles.UARTCOM.hControlSerialPort = hControlSerialPort;
guidata(hObject,handles);


function editNumLanes_Callback(hObject, eventdata, handles)
% hObject    handle to editNumLanes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editNumLanes as text
%        str2double(get(hObject,'String')) returns contents of editNumLanes as a double
handles.scene.numberOfLanes = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function editNumLanes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editNumLanes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function editLaneWidths_Callback(hObject, eventdata, handles)
% hObject    handle to editLaneWidths (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLaneWidths as text
%        str2double(get(hObject,'String')) returns contents of editLaneWidths as a double
handles.scene.laneWidth = str2num(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function editLaneWidths_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLaneWidths (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function editLeftLane_Callback(hObject, eventdata, handles)
% hObject    handle to editLeftLane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLeftLane as text
%        str2double(get(hObject,'String')) returns contents of editLeftLane as a double
handles.scene.leftLineX = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function editLeftLane_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLeftLane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editStopBar_Callback(hObject, eventdata, handles)
% hObject    handle to editStopBar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStopBar as text
%        str2double(get(hObject,'String')) returns contents of editStopBar as a double
handles.scene.stopLineY = repmat(str2double(get(hObject,'String')), 1, handles.scene.numberOfLanes+1);
guidata(hObject,handles);
drawRadarRoom(handles);

% --- Executes during object creation, after setting all properties.
function editStopBar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStopBar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAng_Callback(hObject, eventdata, handles)
% hObject    handle to editAng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAng as text
%        str2double(get(hObject,'String')) returns contents of editAng as a double
handles.angle = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function editAng_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAng (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1


function [P, cliCfg] = parseCfg(cliCfg)
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
        elseif strcmp(C{1}, 'sceneryParam')
            P.gtrackAdvanced.sceneryParam = C;
        elseif strcmp(C{1}, 'gatingParam')
            P.gtrackAdvanced.gatingParam = C;
        elseif strcmp(C{1}, 'stateParam')
            P.gtrackAdvanced.stateParam = C;
        elseif strcmp(C{1}, 'allocationParam')
            P.gtrackAdvanced.allocationParam = C;
        elseif strcmp(C{1}, 'variationParam')
            P.gtrackAdvanced.variationParam = C;
        elseif strcmp(C{1},'trackingCfg')
            P.trackingCfg = C;
        end
    end
%     P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
%                                             P.frameCfg.chirpStartIdx + 1) *...
%                                             P.frameCfg.numLoops;

    P.dataPath.numChirpsPerFrame = P.dataPath.numTxAnt *...
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
    P.dataPath.dopplerIdxToMps = P.dataPath.dopplerResolutionMps*P.frameCfg.numLoops/P.dataPath.numDopplerBins;
    P.dataPath.maxRange = 300 * 0.9 * P.profileCfg.digOutSampleRate /(2 * P.profileCfg.freqSlopeConst * 1e3);
    P.dataPath.maxVelocity = 3e8 / (4*P.profileCfg.startFreq*1e9 *(P.profileCfg.idleTime + P.profileCfg.rampEndTime) * 1e-6 * P.dataPath.numTxAnt);
    
function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
    
function [sphandle] = configureDataPort(comPortNum, bufferSize)
  
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);

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


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on selection change in popupWebcam.
function popupWebcam_Callback(hObject, eventdata, handles)
% hObject    handle to popupWebcam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupWebcam contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupWebcam


% --- Executes during object creation, after setting all properties.
function popupWebcam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupWebcam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
% if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor','white');
% end
% hObject.String = webcamlist();



% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
% if(get(hObject,'Value'))
%     hWebcamSelect = findobj('Tag', 'popupWebcam');
%     handles.camIndex = hWebcamSelect.Value;
%     fprintf('webcam enabled %d', hWebcamSelect.Value)
% else
%     handles.camIndex = -1;
% end
% guidata(hObject,handles);

function [strPorts numPorts] = get_com_ports()
    
    
    command = 'wmic path win32_pnpentity get caption /format:list | find "COM"';
    [status, cmdout] = system (command);
    UART_COM = regexp(cmdout, 'UART\s+\(COM[0-9]+', 'match');
    UART_COM = (regexp(UART_COM, 'COM[0-9]+', 'match'));
    DATA_COM = regexp(cmdout, 'Data\s+Port\s+\(COM[0-9]+', 'match');
    DATA_COM = (regexp(DATA_COM, 'COM[0-9]+', 'match'));
    
    n = length(UART_COM);
    if (n==0)
        errordlg('Error: No Device Detected')
        return
    else
        CLI_PORT = zeros(n,1);
        S_PORT = zeros(n,1);
        strPorts = {};
        for i=1:n
            temp = cell2mat(UART_COM{1,i});
            strPorts{i,1} = temp;
            CLI_PORT(i,1) = str2num(temp(4:end));
            temp = cell2mat(DATA_COM{1,i});
            strPorts{i,2} = temp;
            S_PORT(i,1) = str2num(temp(4:end));
        end

        CLI_PORT = sort(CLI_PORT);
        S_PORT = sort(S_PORT);
        numPorts = [CLI_PORT, S_PORT];
    end

function drawRadarRoom(h)
ax = h.axes1;
if ishandle(ax)
    % clear previously drawn objects
    children = get(ax, 'children');
    delete(children);
    scene = h.scene;
    Params = h.params;
    
    %sensor and scene parameters
    sensor.rangeMax = Params.dataPath.maxRange;
    sensor.rangeMin = 10; % assumed
    sensor.azimuthFoV = 120*pi/180; %120 degree FOV in horizontal direction
    sensor.framePeriod = Params.frameCfg.framePeriodicity; %in ms
    sensor.maxRadialVelocity = Params.dataPath.maxVelocity;
    sensor.radialVelocityResolution = Params.dataPath.dopplerResolutionMps;
    sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);
    scene.azimuthTilt =  h.angle*pi/180;
    scene.lineX = [scene.leftLineX scene.leftLineX+cumsum(scene.laneWidth)];
    scene.numberOfTargetBoxes = scene.numberOfLanes;
    %scene.targetBox = [scene.lineX(1:end-1); scene.stopLineY(1:end-1); scene.laneWidth; 50-scene.stopLineY(1:end-1)]';

    % draw elements
    % draw sensor FOV
    plot(ax, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
    plot(ax, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');

    %draw boundary and static boxes
    
    for n=1:h.box.numBoundary
        b= h.box.boundaryBox(n,:);
        if(numel(b) == 4 && (b(2)-b(1))>0 && (b(4)-b(3))>0)
            rectangle(ax, 'Position', [b(1) b(3) (b(2)-b(1)) (b(4)-b(3))], 'EdgeColor','g', 'LineStyle', '-', 'LineWidth', 0.5);
        end
    end
    for n=1:h.box.numStatic
        b= h.box.staticBox(n,:);
        if(numel(b) == 4 && (b(2)-b(1))>0 && (b(4)-b(3))>0)
            rectangle(ax, 'Position',[b(1) b(3) (b(2)-b(1)) b(4)-b(3)], 'EdgeColor','m', 'LineStyle', '-', 'LineWidth', 0.5);
        end
    end

    % draw lanes
    for xLine = 1:length(scene.lineX)
        if((xLine == 1) || (xLine == length(scene.lineX)))
            line(ax, 'Xdata',[scene.lineX(xLine) scene.lineX(xLine)],'YData',[scene.stopLineY(1) max(h.plotBound.Y)], 'Color','k', 'LineStyle', '-', 'LineWidth', 1);
        else
            line(ax, 'Xdata',[scene.lineX(xLine) scene.lineX(xLine)],'YData',[scene.stopLineY(1) max(h.plotBound.Y)], 'Color','k', 'LineStyle', '--');
        end
    end

    % draw stop line
        line(ax, 'Xdata',[scene.lineX(1) scene.lineX(end)],'YData',[scene.stopLineY(1) scene.stopLineY(1)], 'Color','r', 'LineStyle', '-', 'LineWidth', 2);

    % draw counter line
        line(ax, 'Xdata',[scene.lineX(1) scene.lineX(end)],'YData',[h.scene.startLineY h.scene.startLineY], 'Color','b', 'LineStyle', '--', 'LineWidth', 1);

    xlabel('X coordinate, m');
    ylabel('Y coordinate, m');
    axis equal;
    axis([h.plotBound.X h.plotBound.Y]);
    grid on;
    hold on;
    grid minor;
    h = zeros(3, 1);
    h(1) = plot(NaN,NaN,'-k');
    h(2) = plot(NaN,NaN,'g', 'LineStyle', '-', 'LineWidth', 0.5);
    h(3) = plot(NaN,NaN,'m', 'LineStyle', '-', 'LineWidth', 0.5);
    h(4) = plot(NaN,NaN,'r', 'LineStyle', '-', 'LineWidth', 2);
    h(5) = plot(NaN,NaN,'b', 'LineStyle', '--', 'LineWidth', 1);
    legend(h, 'Radar FOV','Boundary Box','Static Box', 'Stop Bar', 'Start Bar','Location','northeastoutside');
    
end

function editPlotXBound_Callback(hObject, eventdata, handles)
% hObject    handle to editPlotXBound (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editPlotXBound as text
%        str2double(get(hObject,'String')) returns contents of editPlotXBound as a double
handles.plotBound.X = str2num(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);

% --- Executes during object creation, after setting all properties.
function editPlotXBound_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editPlotXBound (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editPlotYBound_Callback(hObject, eventdata, handles)
% hObject    handle to editPlotYBound (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editPlotYBound as text
%        str2double(get(hObject,'String')) returns contents of editPlotYBound as a double
handles.plotBound.Y = str2num(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function editPlotYBound_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editPlotYBound (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selpath = uigetdir();
handles.savePath = selpath; 
hTextSavePath = findobj('Tag', 'textSavePath');
hTextSavePath.String = ['Save to: ' handles.savePath];
[outstring,newpos] = textwrap(hTextSavePath,{hTextSavePath.String});
set(hTextSavePath,'String',outstring,'Position',newpos) 
guidata(hObject,handles);


function editData_Callback(hObject, eventdata, handles)
% hObject    handle to editData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editData as text
%        str2double(get(hObject,'String')) returns contents of editData as a double


% --- Executes during object creation, after setting all properties.
function editData_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editUART_Callback(hObject, eventdata, handles)
% hObject    handle to editUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editUART as text
%        str2double(get(hObject,'String')) returns contents of editUART as a double


% --- Executes during object creation, after setting all properties.
function editUART_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editUART (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editStartCounter_Callback(hObject, eventdata, handles)
% hObject    handle to editStartCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStartCounter as text
%        str2double(get(hObject,'String')) returns contents of editStartCounter as a double
handles.scene.startLineY = str2double(get(hObject,'String'));
guidata(hObject,handles);
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function editStartCounter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStartCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonBrowse.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonBrowse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



%Display Chirp parameters in table on screen
function hTable = displayChirpParams(Params, hTable)

    dat =  {
            'Max Range (m)', Params.dataPath.maxRange;...
            'Max Velocity (m/s)', Params.dataPath.maxVelocity;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Start Frequency (Ghz)', Params.profileCfg.startFreq;...
            'Slope (MHz/us)', Params.profileCfg.freqSlopeConst;...   
            'Samples per chirp', Params.profileCfg.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Frame duration (ms)',  Params.frameCfg.framePeriodicity;...
            'Sampling rate (Msps)', Params.profileCfg.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.profileCfg.freqSlopeConst * Params.profileCfg.numAdcSamples /...
                               Params.profileCfg.digOutSampleRate;...
            'Number of Rx (MIMO)', Params.dataPath.numRxAnt;...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;...
            'Nfft_doppler', Params.dataPath.numDopplerBins;...
            'Nfft_range', Params.dataPath.numRangeBins;};
    columnname =   {'Parameter (Units)', 'Value'};
    columnformat = {'char', 'numeric'};
    
    
    hTable.ColumnName = columnname;
    hTable.Data = dat;
    %hTable.ColumnFormat = columnformat; 
    
function hTable = displayGtrackParams(dat, hTable)
    hTable.Data = dat;
    %hTable.ColumnFormat = columnformat; 


% --- Executes on button press in checkboxOverride.
function checkboxOverride_Callback(hObject, eventdata, handles)
% hObject    handle to checkboxOverride (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkboxOverride


% --- Executes when entered data in editable cell(s) in uitableScenery.
function uitableScenery_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitableScenery (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
hTable = findobj('Tag', 'uitableScenery');
data = hTable.Data;
handles.box.boundaryBox = data(1:2,:);%cellfun(@str2num,data(1:2,:));
handles.box.staticBox = data(3:4,:); %cellfun(@str2num,data(3:4,:));
handles.box.numBoundary = sum(any(handles.box.boundaryBox,2));
handles.box.numStatic = sum(any(handles.box.staticBox,2));
guidata(hObject,handles)
drawRadarRoom(handles);


% --- Executes during object creation, after setting all properties.
function textCOMStatus_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textCOMStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function textSavePath_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textCOMStatus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
