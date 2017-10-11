%clc
%clear all
% close all
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
% m01, m03, v103, v203
datasetidx = 1; dataset_subidx = 2;
data_imucut = load([path, filenames_imucut{datasetidx, dataset_subidx}]);
data_imucut(:,1) = data_imucut(:,1)*10^(-9);
imu_start = data_imucut(1,1) ; imu_end = data_imucut(end,1);
data_vi=load([path filenames_vi{datasetidx}]); % time p_x p_y p_z q_x q_y q_z q_w
% data_filter=load([path 'InavTrajectorym0303.txt']);
data_vi(:,1)=data_vi(:,1)*10^(-9);
data_imu=load([path filenames_imu{datasetidx, dataset_subidx}]); % time  p_x p_y p_z q_  w q_x q_y q_z
data_imu(:,1)=data_imu(:,1)*10^(-9);
slam=data_imu; % read origin_slam data
quat_slam = slam(:, 5:8);
filter=data_vi; %read imu_slam data
quat_filter = filter(:, 5:8);

line2show=2; %x,y,z
% find data_imu index where imu track only starts.

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
origin(:,1) = origin(:,1) * 10^(-9);
%% check timestamp
origin_dur = origin(1,1) - origin(end,1);
filter_dur = filter(1,1) - filter(end, 1);
gap_start_fil_ori = origin(1,1) - filter(1,1);
gap_end_fil_ori = origin(end,1) - filter(end,1);
if (abs(gap_start_fil_ori) >= 20 || abs(gap_end_fil_ori) >= 20)
    disp 'groud truth and your data did not match.'
    return
end

%% attitude alignments
quat_gd = origin(:, 5:8);
quat_filter = filter(:, 5:8);
% quat_filter = quatconj(quat_filter);
% quat_slam = slam(:, 5:8);
R_rb = quat2dcm(quat_gd);
R_wb = quat2dcm(quat_filter);
% time alignment slam with ground truth
disp 'filter first timestamp tmp idx_gd'
[tmp, idx_gd] = min(abs(origin_timestamps - filter(1,1)))
disp 'filter end timestamp tmp02 idx_gdend'
[tmp02, idx_gdend] = min(abs( origin_timestamps - filter(end,1)))
% constant dcm matrix Rrw between slam world and Reference frame(vicon)
disp 'Rrb0'
R_rb0 = quat2dcm(quat_gd(idx_gd, :))
disp 'Rwb0'
R_wb0 = quat2dcm(quat_filter(1,:))
disp 'quat_filter(1,:)'
quat_filter(1,:)
R_rw0 = R_rb0 * R_wb0';
%
R_rb_filter = zeros(size(R_wb));
Rdiff = zeros(size(R_wb));
%% dcm approach.
for i=1:size(R_wb,3)
    R_rb_filter(:,:,i) = R_rw0 * R_wb(:,:,i);
    % ground truth has a difference frequency. using timestamp to align.
    [tmp, idx_tmp] = min(abs(origin_timestamps - filter(i ,1)));
    if (tmp > 0.3)
        disp 'something wrong. in dcm approach'
        [tmp, idx_tmp]
        return
    end
    Rdiff(:, :, i) = R_rb(:,:, idx_tmp)' * R_rb_filter(:,:,i);% bug!
end
 Rdiff(:, :, 1);
quat_diff = dcm2quat(Rdiff);
quat_rb_filter = dcm2quat(R_rb_filter);
[anz_gd, any_gd, anx_gd] = quat2angle(quatnormalize(quat_gd));
[anz_filter, any_filter, anx_filter] = quat2angle(quatnormalize(quat_filter));
[anz_rbfilter, any_rbfilter, anx_rbfilter] = quat2angle(quatnormalize(quat_rb_filter));
%% quat_diff
axangle_qdiff = quat2axang(quat_diff);
figure; plot(filter(:,1), axangle_qdiff(:,4))

%% plotting
figure
subplot(3,1,1)
plot(origin_timestamps(idx_gd:idx_gdend), anz_gd(idx_gd:idx_gdend), 'r')
hold on
plot(origin_timestamps(idx_gd:idx_gdend), any_gd(idx_gd:idx_gdend), 'g')
plot(origin_timestamps(idx_gd:idx_gdend), anx_gd(idx_gd:idx_gdend), 'b')
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

%% abs plotting

figure
subplot(3,1,1)
plot(origin_timestamps(idx_gd:idx_gdend), abs(anz_gd(idx_gd:idx_gdend)), 'r')
hold on
plot(origin_timestamps(idx_gd:idx_gdend), abs(any_gd(idx_gd:idx_gdend)), 'g')
plot(origin_timestamps(idx_gd:idx_gdend), abs(anx_gd(idx_gd:idx_gdend)), 'b')
title('ground truth')
subplot(3,1,2)
plot(filter(:,1), abs(anz_rbfilter), 'r')
hold on
plot(filter(:,1), abs(any_rbfilter), 'g')
plot(filter(:,1), abs(anx_rbfilter), 'b')
title('aligned')

subplot(3,1,3)
plot(filter(:,1), abs(anz_filter), 'r')
hold on
plot(filter(:,1), abs(any_filter), 'g')
plot(filter(:,1), abs(anx_filter), 'b')
title('from alg')
hold off
