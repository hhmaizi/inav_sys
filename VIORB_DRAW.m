%clc
%clear all
close all

%% data read
path = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\test0929\'; % #timestamp p_RS_R_x  p_RS_R_y p_RS_R_z q_RS_w q_RS_x q_RS_y q_RS_z v_RS_R_x v_RS_R_y v_RS_R_z
filenames_vi = {'M01_FT_VI.txt', 'M03_FT_VI.txt', 'V103_FT_VI.txt', 'V203_FT_VI.txt'};
filenames_imu = {'M01_FT_IMU01_whole.txt', 'M01_FT_IMU01_05.txt', 'M01_FT_IMU02_05.txt', 'M01_FT_IMU03_05.txt', 'M01_FT_IMU04_05.txt';...
    '','','','','';...
   'V103_FT_VI.txt', 'V103_FT_IMU01_05.txt', 'V103_FT_IMU02_05.txt','V103_FT_IMU03_05.txt','V103_FT_IMU04_05.txt';...
   'V203_FT_VI.txt', 'V203_FT_IMU01_05.txt', 'V203_FT_IMU02_05.txt','V203_FT_IMU03_05.txt', 'V203_FT_IMU04_05.txt'};

filenames_imucut = {'M01_INAV_IMU01_whole.txt', 'M01_INAV_IMU01_05.txt', 'M01_INAV_IMU02_05.txt', 'M01_INAV_IMU03_05.txt', 'M01_INAV_IMU04_05.txt';...
    '','','','','';...
   'V103_INAV_VI.txt', 'V103_INAV_IMU01_05.txt', 'V103_INAV_IMU02_05.txt','V103_INAV_IMU03_05.txt','V103_INAV_IMU04_05.txt';...
   'V203_INAV_VI.txt', 'V203_INAV_IMU01_05.txt', 'V203_INAV_IMU02_05.txt','V203_INAV_IMU03_05.txt', 'V203_INAV_IMU04_05.txt'};
datasetidx = 4; dataset_subidx = 2;
data_imucut = load([path, filenames_imucut{datasetidx, dataset_subidx}]);
data_imucut(:,1) = data_imucut(:,1)*10^(-9);
imu_start = data_imucut(1,1) ; imu_end = data_imucut(end,1);
data_vi=load([path filenames_vi{datasetidx}]); % time p_x p_y p_z q_x q_y q_z q_w
% data_filter=load([path 'InavTrajectorym0303.txt']);
data_vi(:,1)=data_vi(:,1)/1000/1000/1000;
data_imu=load([path filenames_imu{datasetidx, dataset_subidx}]); % time  p_x p_y p_z q_  w q_x q_y q_z
data_imu(:,1)=data_imu(:,1)*10^(-9);

line2show=2; %x,y,z
% find data_imu index where imu track only starts.
data_optimize=load('PoseOptimization.txt');
slam=data_imu; % read origin_slam data
quat_slam = slam(:, 5:8);
filter=data_vi; %read imu_slam data
quat_filter = filter(:, 5:8);
%%
% disp 'let us go'
[imu_start, imu_end] - slam(1,1)
[slam(1,1), slam(end,1)] - slam(1,1)
[filter(1,1), filter(end, 1)] - slam(1,1)
idxstart_slam = find(slam(:,1) == imu_start, 1);
slam(idxstart_slam,1) - imu_start
idxend_slam = find(slam(:,1) == imu_end, 1);
slam(idxend_slam, 1 ) - imu_end
% idxstart_filter = find(filter(:,1) == imu_start, 1);
[tmp, idxstart_filter] = min(abs(filter(:,1)-imu_start));
filter(idxstart_filter,1) - imu_start
idxend_filter = find(filter(:,1) == imu_end, 1);
filter(idxend_filter, 1) - imu_end
%%
% filepath = '\\10.10.194.34\foaa\SLAM\DATASET\WEB_DATA\EUROC_DATASET_1\MH_03_medium\mav0\state_groundtruth_estimate0\';
% filename = uigetfile([filepath, '*.csv'],'Select the .csv file containing ground_truth pose');
origin=data; %read ground_truth Tiw, get data from ground truth csv
quat_gd = origin(:, 5:8);
origin_timestamps = origin(:,1)*10^(-9);
% idxstart_origin = find( origin_timestamps >= imu_start, 1);
[tmp, idxstart_origin] = min(abs(origin_timestamps - imu_start));
% idxend_origin = find(origin_timestamps >= imu_end, 1);
[tmp, idxend_origin] = min(abs(origin_timestamps - imu_end));

pci = [-0.0216,-0.0647,0.0098];
% TBS in cam0
rci=[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0];
qci =  quatconj(dcm2quat(rci(1:3,1:3))); %左乘矩阵变换，故而默认右乘情况下要加个转置
qiw=[0.5341,-0.1530,-0.8274,-0.0822];
piw=[4.6883,-1.7869,0.7833];
imu_cam=-inv(rci(1:3,1:3))*pci';
%% slam2world
slam_worldimu=[];
slam_quat=[];

%% attitude alignments
quat_gd = origin(:, 5:8);
quat_filter = filter(:, 5:8);
quat_filter = quatconj(quat_filter);
% quat_slam = slam(:, 5:8);
R_rb = quat2dcm(quat_gd);
R_wb = quat2dcm(quat_filter);
% time alignment slam with ground truth
disp 'tmp idx_gd'
[tmp, idx_gd] = min(abs(origin_timestamps - filter(1,1)))
disp 'tmp02 idx_gdend'
[tmp02, idx_gdend] = min(abs( origin_timestamps - filter(end,1)));
% constant dcm matrix Rrw between slam world and Reference frame(vicon)
R_rb0 = quat2dcm(quat_gd(idx_gd, :));
R_wb0 = quat2dcm(quat_filter(1,:));
quat_filter(1,:)
R_rw0 = R_rb0 * R_wb0';
%
R_rb_filter = zeros(size(R_wb));
Rdiff = [];
for i=1:size(R_wb,3)
    R_rb_filter(:,:,i) = R_rw0 * R_wb(:,:,i);
    % ground truth has a difference frequency. using timestamp to align.
    [tmp, idx_tmp] = min(abs(origin_timestamps - filter(i ,1)));
    if (tmp > 0.002)
        disp 'something wrong.'
        [tmp, dix_tmp]
    end
    Rdiff(:, :, i) = R_rb(:,:, idx_tmp)' * R_rb_filter(:,:,i);% bug!
end
 Rdiff(:, :, 1);
quat_diff = dcm2quat(Rdiff);
quat_rb_filter = dcm2quat(R_rb_filter);
[anz_gd, any_gd, anx_gd] = quat2angle(quatnormalize(quat_gd));
[anz_filter, any_filter, anx_filter] = quat2angle(quat_filter);
[anz_rbfilter, any_rbfilter, anx_rbfilter] = quat2angle(quat_rb_filter);
%% quat_diff
axangle_qdiff = quat2axang(quat_diff);
figure; plot(filter(:,1), axangle_qdiff(:,4))

%% plotting
figure
subplot(3,1,1)
plot(origin_timestamps(idx_gd:idx_gdend), abs(anz_gd(idx_gd:idx_gdend)), 'r')
hold on
plot(origin_timestamps(idx_gd:idx_gdend), abs(any_gd(idx_gd:idx_gdend)), 'g')
plot(origin_timestamps(idx_gd:idx_gdend), abs(anx_gd(idx_gd:idx_gdend)), 'b')
title('ground truth')
% subplot(2,1,1)
% plot(origin_timestamps(idx_gd:idx_gdend), anz_gd(idx_gd:idx_gdend), 'r')
% hold on
% plot(origin_timestamps(idx_gd:idx_gdend), any_gd(idx_gd:idx_gdend), 'g')
% plot(origin_timestamps(idx_gd:idx_gdend), anx_gd(idx_gd:idx_gdend), 'b')
subplot(3,1,2)
plot(filter(:,1), anz_rbfilter, 'r')
hold on
plot(filter(:,1), any_rbfilter, 'g')
plot(filter(:,1), anx_rbfilter, 'b')
title('aligned')

subplot(3,1,3)
plot(filter(:,1), anz_filter, 'r')
hold on
plot(filter(:,1), any_filter, 'g')
plot(filter(:,1), anx_filter, 'b')
title('from alg')
hold off

%%
%begin frame
for i=1:1:length(origin(:,1)-1)
   if(((origin(i,1)/1000/1000/1000-slam(1,1))) > 0 && (origin(i+1,1)/1000/1000/1000-slam(1,1)) > 0)
        fprintf('timestamp error!!\r\n');
   end
   if(abs((origin(i,1)/1000/1000/1000-slam(1,1)))<abs((origin(i+1,1)/1000/1000/1000-slam(1,1))))
       piw=origin(i,2:4);
       qiw=origin(i,5:8);
       i
       break;
   end
end
%slam cam_vision -> imu_world
for i=1:1:length(slam(:,1))
    imu_vision=quatrotate(quatconj(slam(i,5:8)),imu_cam')+slam(i,2:4); %frame imu pos in vision coordinate 
    slam_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
    slam_worldimu(end,:)=slam_worldimu(end,:)+pci;
    slam_worldimu(end,:)=quatrotate(quatconj(qiw), slam_worldimu(end,:))+piw;
end
for i=1:1:length(slam(:,1))
    slam_quat(end+1,:)=quatmultiply(qiw,quatmultiply(qci,quatmultiply(slam(i,5:8),quatconj(qci))));
end
%tmp version
% for i=1:1:length(slam(:,1))
%     imu_vision=quatrotate(quatconj(slam(i,[8,5,6,7])),imu_cam')+slam(i,2:4); %frame imu pos in vision coordinate 
%     slam_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
%     slam_worldimu(end,:)=slam_worldimu(end,:)+pci;
%     slam_worldimu(end,:)=quatrotate(quatconj(qiw), slam_worldimu(end,:))+piw;
% end

%% filter2world
filter_worldimu=[];
filter_quat=[];
origin_frame_begin=1;
if 1
%begin frame
for i=1:1:length(origin(:,1))
    if(((origin(i,1)/1000/1000/1000-filter(1,1))) > 0 && (origin(i+1,1)/1000/1000/1000-filter(1,1)) > 0)
        fprintf('timestamp error!!\r\n');
    end
   if(abs((origin(i,1)/1000/1000/1000-filter(1,1)))<abs((origin(i+1,1)/1000/1000/1000-filter(1,1))))
       piw=origin(i,2:4);
       qiw=origin(i,5:8);
       i
       origin_frame_begin=i;
       break;
   end
end
%filter cam_vision -> imu_world
% for i=1:1:length(filter(:,1))
%     imu_vision=quatrotate(quatconj(filter(i,[8,5,6,7])),imu_cam')+filter(i,2:4);
%     filter_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
%     filter_worldimu(end,:)=filter_worldimu(end,:)+pci;
%     filter_worldimu(end,:)=quatrotate(quatconj(qiw), filter_worldimu(end,:))+piw;
% end
% for i=1:1:length(filter(:,1))
%     filter_quat(end+1,:)=quatmultiply(qiw,quatmultiply(qci,quatmultiply(filter(i,[8,5,6,7]),quatconj(qci))));pi
% end
%tmp version
for i=1:1:length(filter(:,1))
    imu_vision=quatrotate(quatconj(filter(i,5:8)),imu_cam')+filter(i,2:4);
    filter_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
    filter_worldimu(end,:)=filter_worldimu(end,:)+pci;
    filter_worldimu(end,:)=quatrotate(quatconj(qiw), filter_worldimu(end,:))+piw;
end
for i=1:1:length(filter(:,1))
    filter_quat(end+1,:)=quatmultiply(qiw,quatmultiply(qci,quatmultiply(filter(i,5:8),quatconj(qci))));
end

else
    for i=1:1:length(filter(:,1))
        filter_worldimu(end+1,:)=filter(i,2:4);
    end
end
%% EKF2world
%imu imu_world
% for i=1:1:length(predict(:,1))
%     predict(i,2:4)=quatrotate(quatconj(qiw),predict(i,2:4))+piw;
% end
%origin imu_world -> cam_world
% for i=1:1:length(origin(:,1))
%     origin(:,2:4)=quatrotate(quatconj(origin(:,5:8)),pci)+origin(:,2:4);
% end

%% variance calculation
filter_pointer=1;
filter_nocmp=0;
filter_variance=0;
filter_angle_diff=0;
slam_pointer=1;
slam_nocmp=0;
slam_variance=0;
slam_angle_diff=0;
count=0;
a=0;
for i=1:1:length(origin(:,1))-1
     t=double(origin(i,1)/1000/1000/1000);
     t_plus1=double(origin(i+1,1)/1000/1000/1000);
    if(filter_pointer<=length(filter(:,1)))
        while(filter_pointer<=length(filter(:,1)) && (t-filter(filter_pointer,1)) > 0 && (t_plus1-filter(filter_pointer,1)) > 0)
            filter_pointer=filter_pointer+1;
            filter_nocmp=filter_nocmp+1;
        end
        if(filter_pointer<=length(filter(:,1)) && abs(t-filter(filter_pointer,1))<abs(t_plus1-filter(filter_pointer,1)))
            filter_variance=filter_variance+(norm(filter_worldimu(filter_pointer,1:3)-origin(i,2:4)))^2;
             angle_filter=acosd(abs(filter_quat(filter_pointer,1)));
             angle_origin=acosd(abs(origin(i,5)));
             filter_angle_diff=filter_angle_diff+abs(angle_filter-angle_origin);
            filter_pointer=filter_pointer+1;
        end
    end
    if(slam_pointer<=length(slam(:,1)))
        while((t-slam(slam_pointer,1)) > 0 && (t_plus1-slam(slam_pointer,1)) > 0)
            slam_pointer=slam_pointer+1;
            slam_nocmp=slam_nocmp+1;
        end
        if(abs(t-slam(slam_pointer,1))<abs(t_plus1-slam(slam_pointer,1)))
            slam_variance=slam_variance+(norm(slam_worldimu(slam_pointer,1:3)-origin(i,2:4)))^2;
             angle_slam=acosd(abs(slam_quat(slam_pointer,1)));
             angle_origin=acosd(abs(origin(i,5)));
             slam_angle_diff=slam_angle_diff+abs(angle_slam-angle_origin);
            slam_pointer=slam_pointer+1;
        end
    end
end
filter_variance=sqrt(filter_variance/(filter_pointer-2-filter_nocmp))
slam_variance=sqrt(slam_variance/(slam_pointer-2-slam_nocmp))
filter_angle_diff=filter_angle_diff/(filter_pointer-1-filter_nocmp)
slam_angle_diff=slam_angle_diff/(slam_pointer-1-slam_nocmp)

sum=0;
for i=2:1:length(filter_worldimu(:,1))
    sum=sum+norm(filter_worldimu(i,1:3)-filter_worldimu(i-1,1:3));
end
loop_dis=norm(filter_worldimu(end,1:3)-filter_worldimu(1,1:3));
percent=loop_dis/sum

% op_length=length(data_optimize(:,1));
% op_diff=[];
% for i=1:1:op_length
%     if(data_optimize(i,1)~=data_optimize(i-1,1))
%        op_diff(end+1,1)=data_optimize(i-1,1);
%        a_dis=norm(data_optimize(i-1,3:5));
%        b_dis=norm(data_optimize(i-2,3:5));
%        op_diff(end,2)=abs(a_dis-b_dis);
%     end
% end
data_record=[];
for i=1:1:length(data_optimize(:,1))
    imu_vision=data_optimize(i,3:5);%quatrotate(quatconj(data_optimize(i,6:9)),imu_cam')+data_optimize(i,3:5);
    data_record(end+1,:)=quatrotate(quatconj(qci),imu_vision);
    data_record(end,:)=data_record(end,:)+pci;
    data_record(end,:)=quatrotate(quatconj(qiw), data_record(end,:))+piw;
    data_optimize(i,3:5)=data_record(i,:);
    data_optimize(i,10) = data_optimize(i,10)/1000/1000/1000;
end
op_length=length(data_optimize(:,1));
opt_1=[];
opt_2=[];
opt_3=[];
for i=4:1:op_length
    if(data_optimize(i,1)~=data_optimize(i-1,1))
       if(data_optimize(i-1,2)==2)
           opt_1(end+1,1)=data_optimize(i-1,10);
           opt_1(end,2)=data_optimize(i-1,line2show+2);
           if(data_optimize(i-1,1)==data_optimize(i-2,1))
               opt_2(end+1,1)=data_optimize(i-2,10);
               opt_2(end,2)=data_optimize(i-2,line2show+2);
               opt_3(end+1,1)=data_optimize(i-3,10);
               opt_3(end,2)=data_optimize(i-3,line2show+2);
           end       
       else
           opt_2(end+1,1)=data_optimize(i-1,10);
           opt_2(end,2)=data_optimize(i-1,line2show+2);
           opt_3(end+1,1)=data_optimize(i-2,10);
           opt_3(end,2)=data_optimize(i-2,line2show+2);
       end
    end
end
%% figure draw
figure(1)
plot(filter(1:end,1),filter_worldimu(1:end,line2show),'b');
hold on
plot(slam(:,1),slam_worldimu(:,line2show),'g');
hold on
plot(slam(idxstart_slam : idxend_slam, 1), slam_worldimu(idxstart_slam : idxend_slam, line2show), 'm');% 'LineWidth'
plot(double(origin(1:end,1))/1000/1000/1000,origin(1:end,line2show + 1),'r');
legend('vio tracking','vio + imu tracking', 'imu only tracking', 'groundtruth');

%% manually align vio tracking and imu only tracking.
shift_slam_filter = filter_worldimu(idxstart_filter, 1:3) - slam_worldimu(idxstart_slam, 1:3);

figure(2)
plot(filter(idxstart_filter: idxend_filter, 1), filter_worldimu(idxstart_filter: idxend_filter, line2show),'b');
hold on
plot(slam(idxstart_slam : idxend_slam, 1),...
    slam_worldimu(idxstart_slam : idxend_slam, line2show) + shift_slam_filter(line2show), 'g');% 'LineWidth'
plot(double(origin(idxstart_origin:idxend_origin, 1))/1000/1000/1000,origin(idxstart_origin:idxend_origin, line2show + 1),'r');
legend('vio tracking', 'imu only tracking', 'groundtruth');
title('local inspection')

% figure(2)
% plot(filter(idxstart_filter: idxend_filter, 1), filter_worldimu(idxstart_filter: idxend_filter, line2show),'b');
% hold on
% plot(slam(idxstart_slam : idxend_slam, 1), slam_worldimu(idxstart_slam : idxend_slam, line2show), 'g');% 'LineWidth'
% plot(double(origin(idxstart_origin:idxend_origin, 1))/1000/1000/1000,origin(idxstart_origin:idxend_origin, line2show + 1),'r');
% legend('vio tracking', 'imu only tracking', 'groundtruth');
% title('local inspection')


%%plot3d
% scatter3(slam_worldimu(:,1),slam_worldimu(:,2),slam_worldimu(:,3),'b');
% hold on
% scatter3(origin(:,2),origin(:,3),origin(:,4),'r');

