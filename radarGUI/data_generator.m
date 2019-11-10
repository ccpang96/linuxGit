%% UKF bicycle test
clear all
close all

%% ��ʼ��

% ���ó�ʼλ��

%p_x_start = 0.6;
%p_y_start = 0.6;

p_x_start = 10;
p_y_start = 10;

base_timestamp = 0; % ����ʱ�� ��λ��us 
delta_t_us = 0.5*1e5; % ʱ��̶�t ��λ��us
delta_t_sec = delta_t_us / 1e6; %ʱ��̶�t ��λ��s
round_time_sec = 25; %����

n_z = round_time_sec/delta_t_sec;  % ���������ܵ���
n_x = 5;  % ״̬ά��

%��ʵֵ
GT = zeros(n_x,n_z); % ��ʵֵ: p1 p2 v_abs yaw yaw_dot v1 v2  x����λ�� y����λ�� �����ٶ�v �Ƕ�yaw �Ƕȱ仯��yaw_dot x�����ٶ� y�����ٶ�
%����ͺ��ײ��״�Ĳ���ֵ
Z_l = zeros(3,n_z); % �����״����ֵ: pos1, pos2 time                 
Z_r = zeros(4,n_z); % ���ײ��״����ֵ: r, phi, r_dot time

% ������ʼֵ
GT(1,1) =  p_x_start;
GT(2,1) =  p_y_start;

%�������
std_las1 = 0.15;
std_las2 = 0.15;

std_radr = 0.3;
std_radphi = 0.03;
std_radrd = 0.3;

for k = 1:n_z
    timestamp = base_timestamp+(k-1)*delta_t_us; %����0.05s�ɼ�һ������
    round_time_sec = 25; % ����
    % ʱ��
    GT(8,k) = timestamp;
    % �ı��ٶ�
    time_sec = k*delta_t_us/1e6;
 
  
   %��0������
    %GT(3,k) = 2 + 1.6*sin(2*2*pi/round_time_sec*time_sec); %�����ٶ�
    % �Ƕȱ仯��
    %GT(5,k) = 1.55*sin(2*pi/round_time_sec*time_sec);  %�����Ƕȱ任��
    
    
    %��һ�����ݣ�
    %GT(3,k) = 100; %�ٶ���100
    %GT(5,k) = 0.05 * (2*pi/round_time_sec*time_sec); %����radarGUI����ӵ�

    GT(3,k) = 100 + 1.6*sin(2*2*pi/round_time_sec*time_sec); %�����ٶ�
    % �Ƕȱ仯��
    GT(5,k) = 0.65*sin(2*pi/round_time_sec*time_sec);  %�����Ƕȱ任��
    
    % yaw rate(�Ǽ��ٶ�)
    %GT(5,k) = 0.55*sin(2*pi/round_time_sec*time_sec);
end

Z_l(3,1)=base_timestamp;
Z_r(4,1)=base_timestamp;

%% ��һ������ֵ (�������) 
% �����״�
Z_l(1,1) = GT(1,1) + normrnd(0, std_las1) ;
Z_l(2,1) = GT(2,1) + normrnd(0, std_las2) ;
Z_l(3,1) = GT(8,1) ;  % ʱ��

% ���ײ��״����ֵ
p1 = GT(1,1);
p2 = GT(2,1);
v =  GT(3,1);
yaw = GT(4,1);

v1 = GT(3,1) * cos(GT(4,1));
v2 = GT(3,1) * sin(GT(4,1));

Z_r(1,1) = sqrt(p1^2 + p2^2) + normrnd(0, std_radr) ;                        %r
Z_r(2,1) = atan2(p2,p1) + normrnd(0, std_radphi) ;                           %phi
Z_r(3,1) = (p1*v1 + p2*v2 ) / sqrt(p1^2 + p2^2) + normrnd(0, std_radrd) ;    %r_dot
Z_r(4,1) = GT(8,1) ;  % time

GT(6,1) = cos(yaw)* v; % v_x
GT(7,1) = sin(yaw)* v; % v_y
%%
% ��λ�Ͳ���
for k = 2:n_z
    
    delta_t_s = (GT(8,k) - GT(8,k-1))/1e6;
    
    p1 = GT(1,k-1);
    p2 = GT(2,k-1);
    v = GT(3,k-1);
    yaw = GT(4,k-1);
    yaw_dot = GT(5,k-1);
    
    if abs(yaw_dot) > 0.001   
        p1_p = p1 + v/yaw_dot * ( sin (yaw + yaw_dot*delta_t_s) - sin(yaw)); 
        p2_p = p2 + v/yaw_dot * ( cos(yaw) - cos(yaw+yaw_dot*delta_t_s) );
    else
        p1_p = p1 + v*delta_t_s*cos(yaw);
        p2_p = p2 + v*delta_t_s*sin(yaw);
    end
    v_p = v;
    yaw_p = yaw + yaw_dot*delta_t_s;
    yaw_dot_p = yaw_dot;
    
    GT(1,k) = p1_p;
    GT(2,k) = p2_p;
    GT(4,k) = yaw_p;

    % �����״����ֵ    
    Z_l(1,k) = GT(1,k) + normrnd(0, std_las1) ;
    Z_l(2,k) = GT(2,k) + normrnd(0, std_las2) ;
    Z_l(3,k) = GT(8,k) ;  % time

    % ���ײ��״����ֵ
    p1 = GT(1,k);  %vx
    p2 = GT(2,k);  %vy
    v = GT(3,k);   %v
    yaw = GT(4,k); %�Ƕ�

    v1 = GT(3,k) * cos(GT(4,k));
    v2 = GT(3,k) * sin(GT(4,k));

    Z_r(1,k) = sqrt(p1^2 + p2^2) + normrnd(0, std_radr) ;  %r
    Z_r(2,k) = atan2(p2,p1) + normrnd(0, std_radphi) ;   %�Ƕȣ�������ڵ�һ���ޣ�����0~pi/2������ǵڶ����ޣ�����pi/2 ~ pi; ����ǵ������ޣ�����-pi~-pi/2; ����ǵ������ޣ�����-pi/2 ~0; 
    %�����ݽ���һ��ת��
    %{
    if Z_r(2,k) > 0 %λ�ڵ�һ�͵ڶ����� 
        Z_r(2,k) = 180 * Z_r(2,k) / pi;
    else 
        Z_r(2,k) =  360 - 180 * Z_r(2,k) / pi;
    end 
    %}
    Z_r(3,k) = (p1*v1 + p2*v2 ) / sqrt(p1^2 + p2^2) + normrnd(0, std_radrd) ;  %r_dot
    Z_r(4,k) = GT(8,k) ;  % time
    
    GT(6,k) = cos(yaw)* v; % v_x
    GT(7,k) = sin(yaw)* v; % v_y
end

%% ������ʵֵ�ļ�
GTT=GT';

%��ʵֵ - p1 p2 v_abs yaw yaw_dot v1 v2
fileID = fopen('Z:\sister\radarGUI\generator_data\obj_pose-laser-radar-synthetic-gt.txt','w');
[nrows,ncols] = size(GTT);
for row = 1:nrows
    fprintf(fileID,'%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n',GTT(row,1:8));
end
fclose(fileID);

%% ���ɲ��������ļ� 
%add timestamps
%L    p1    p2    timestamp=
%R    r theta    r_dot    timestamp

%laser structure
Z_lT = Z_l';
N_L =size(Z_lT,1);
v_z = zeros(N_L,1); %vector of zeros
Z_lT = [Z_lT, v_z];
%create a laser cell array - mixed numbers and characters
C_l = cell(N_L,1);
C_l(:) = {'L'};
C_l = [ C_l, num2cell(Z_lT)];

%radar structure
Z_rT = Z_r';
N_R =size(Z_rT,1);
%create a radar cell array - mixed numbers and characters
C_r = cell(N_R,1);
C_r(:) = {'R'};
C_r = [ C_r, num2cell(Z_rT)];


AB ={};
for i = 1:size(Z_lT,1)
    %�Ӹ��Ծ����в������ֵ��ȥ
    AB = [AB; C_l(i,:)];
    AB = [AB; C_r(i,:)];
end


%% ������ݸ�ʽ���ںϵķ�ʽ��һ�������״����ݡ�һ�����ײ��״�����
fileID = fopen('Z:\sister\radarGUI\generator_data\laser_radar.txt','w');
[nrows,ncols] = size(AB);
row_gt = int16(1);

for row = 1:nrows
    if( AB{row,1} == 'L' &&  mod(row,4) == 1 )
        
        %����ֵ
        %fprintf('%s\t%.6f\t%.6f\t%d\n',AB{row,1:ncols-1});
        fprintf(fileID,'%s\t%.6f\t%.6f\t%d\t',AB{row,1:ncols-1});
        %��ʵֵλ�ú��ٶ�
        %fprintf('%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy 
        fprintf(fileID,'%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy 
        
        %�Ƕ�yaw�ͽǶȱ仯��yaw_rate 
        % fprintf(fileID,'%d\t%d\n',GTT(row_gt, [4 5]));
        
    elseif( AB{row,1} == 'R' &&  mod(row,4) == 0 )
        
        %ģ�����ֵ
       % fprintf('%s\t%.6f\t%.6f\t%.6f\t%d\n',AB{row,:});
        fprintf(fileID,'%s\t%.6f\t%.6f\t%.6f\t%d\t',AB{row,:});
        %��ʵλ�ú��ٶ�ֵ
       % fprintf('%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy ;
        fprintf(fileID,'%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy ;
         
    end
    if(mod(row,2) == 0) %һ����ʵֵ�ֱ��Ӧ�״����ݺͼ����״�����
        row_gt = row_gt + 1;
    end;
end

fclose(fileID); 


%% ������ݸ�ʽ��ֻ��������״�����
fileID = fopen('Z:\sister\radarGUI\generator_data\input_lidar.txt','w');
[nrows,ncols] = size(AB);
row_gt = int16(1);

for row = 1:nrows
    if( AB{row,1} == 'L' )
        
        %����ֵ
      
        fprintf(fileID,'%s\t%.6f\t%.6f\t%d\t',AB{row,1:ncols-1});
        %��ʵֵλ�ú��ٶ�
        fprintf(fileID,'%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy 
    end
    if(mod(row,2) == 0) %һ����ʵֵ�ֱ��Ӧ�״����ݺͼ����״�����
        row_gt = row_gt + 1;
    end;
end

fclose(fileID); 


%% ������ݸ�ʽ��ֻ������ײ��״�����
fileID = fopen('Z:\sister\radarGUI\data\input_radar.txt','w');
[nrows,ncols] = size(AB);
row_gt = int16(1);

for row = 1:nrows
    if( AB{row,1} == 'R' )
        
        %ģ�����ֵ
       % fprintf('%s\t%.6f\t%.6f\t%.6f\t%d\n',AB{row,:});
        %fprintf(fileID,'%s\t%.6f\t%.6f\t%.6f\t%d\t',AB{row,:});
        fprintf(fileID,'%d\t',AB{row,5});
        fprintf(fileID,'%.6f\t%.6f\t%.6f\t',AB{row,2:4});
        %��ʵλ�ú��ٶ�ֵ
       % fprintf('%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy ;
       % fprintf(fileID,'%.6f\t%.6f\t%.6f\t%.6f\n',GTT(row_gt, [1 2 6 7]));  %��ʵpx py vx vy ;
       fprintf(fileID,'%.6f\t%.6f\n',GTT(row_gt,[1,2]));
    end
    if(mod(row,2) == 0) %һ����ʵֵ�ֱ��Ӧ�״����ݺͼ����״�����
        row_gt = row_gt + 1;
    end;
end

fclose(fileID); 




% �������ռ䱣������
save('Z:\sister\radarGUI\generator_data\bicycle_data.mat') 

figure(2)
hold on;
plot(GT(1,:), GT(2,:), '-og'); 

xlabel('x');
ylabel('y');
axis equal
legend('GT')

%%
figure(1)
hold on;
plot(GT(1,:), '.-k');  %py
plot(GT(2,:), '.-b');  %py
plot(GT(3,:), '.-g');  %v
plot(GT(4,:), '.-r');  %yaw
plot(GT(5,:), '.-m');  %yawrate 
plot(diff(GT(3,:))/delta_t_sec, '-c'); %acc
plot(diff(GT(5,:))/delta_t_sec, '.c'); %yawacc
legend('px','py', 'v', 'yaw', 'yawrate', 'acc', 'yawacc')

