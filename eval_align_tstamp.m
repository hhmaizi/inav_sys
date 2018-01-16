%clc
close all
clear 
%% read the data
% viorb_readdata;
%% params
Tbc = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0];

%%
path_data = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\test1023\'; % #timestamp p_RS_R_x  p_RS_R_y p_RS_R_z q_RS_w q_RS_x q_RS_y q_RS_z v_RS_R_x v_RS_R_y v_RS_R_z
filenames_slam = {'M01_FT_VI.txt', 'M03_FT_VI.txt', 'V103_FT_VI.txt', 'V203_FT_VI.txt'};
data_id = 1; duration = 2.0;
% 	1. V103_FT_IMU01_05.txt 1403715930.0==>1403715935.0;
% 	2. V103_FT_IMU02_05.txt	1403715950.0==>1403715955.0;
% 	3. V103_FT_IMU03_05.txt 1403715970.0==>1403715975.0;
vst_tstamp = [1403715930.0; 1403715950.0; 1403715970.0; 1403715925.0];
data_fname = 'FrameTrajectory.txt';
data_gtruth = load('v103_ground_truth.csv');
data_slam0 = load([path_data, 'track without imu\', data_fname]);
data_inav0 = load([path_data  sprintf('%03d\\', data_id) data_fname]);

[data_slam, data_inav] = align_timestamp(data_slam0, data_inav0);
% data_slam = load([path_data, filenames_slam{1}]);
% data_slam = load('.\data\mh01.txt');
% data_slam = load('.\data\mh01_test.txt');
% find data_imu index where imu track only starts.
%% cut your data so inav and slam concord
% timestamp_end = 1403715936.0;
% [tmp_diff, idx1_slam] = min(abs(data_slam(:,1) - timestamp_end * 1.0e9));
% [tmp_diff, idx1_inav] = min(abs(data_inav(:,1) - timestamp_end * 1.0e9));
% data_slam = data_slam(1: idx1_slam, :);
% data_inav = data_inav(1: idx1_inav, :);

%% find corresponding ground truth to data_slam, according to timestamp
[data_slam, data_gtruth4slam] = align_time_stamp(data_slam, data_gtruth);
[data_inav, data_gtruth4inav] = align_time_stamp(data_inav, data_gtruth);
%% inav start timestamp in data_slam
% st_gtruth = data_gtruth(1,1) * 1.0e-9;
% st_tstamp_inav = 1403715930.0 ;
% end_tstamp_inav = 1403715935.0;
% [t_diff_start, idx_slam_inav_start] = min(abs(data_slam(:,1) - st_tstamp_inav));
% [t_diff_end, idx_slam_inav_end] = min(abs( data_slam(:,1) - end_tstamp_inav ));

%% test
% figure
% histogram(data_gtruth4slam(:,1) - data_slam(:,1))

%% comparing data_slam and data_inav
%
% pos comparing
blocal_sinav = (data_slam(:,1) > vst_tstamp(data_id)) & ...
    (data_slam(:,1) < vst_tstamp(data_id) + duration) ;
idxs_inav = 1:size(data_slam,1);
idxs_inav = idxs_inav(blocal_sinav);
%% position comparing
figure;
subplot(3,1,1)
plot(data_slam(idxs_inav,1), data_slam(idxs_inav, 2) - data_inav(idxs_inav,2), 'b')
title('x diff')
subplot(3,1,2)
plot(data_slam(idxs_inav,1), data_slam(idxs_inav, 3) - data_inav(idxs_inav,3), 'g')
title('y diff')
subplot(3,1,3)
plot(data_slam(idxs_inav,1), data_slam(idxs_inav, 3) - data_inav(idxs_inav,3), 'r')
title('z diff')
suptitle('slam inav position comparing')
%
%% attitude comparing
qdiff_slam_inav = quatmultiply(data_slam(idxs_inav, 5:8), ...
    quatconj(data_inav(idxs_inav, 5:8)));
[anz_dsinav, any_dsinav, anx_dsinav] = quat2angle( qdiff_slam_inav);
figure;
subplot(3,1,1)
plot(data_slam(idxs_inav,1), anx_dsinav, 'b')
title('anx diff')
subplot(3,1,2)
plot(data_slam(idxs_inav,1), any_dsinav, 'g')
title('any diff')
subplot(3,1,3)
plot(data_slam(idxs_inav,1), anz_dsinav, 'r')
title('anz diff')
suptitle('slam inav attitude comparing')

%% transform slam to Reference frame
% preparing the raw data
quat_gd = data_gtruth4slam(:, 5:8);
pos_gd = data_gtruth4slam(:,2:4);
quat_slam_bak = data_slam(:, 5:8); % body(imu) not cam0
% quat_cam_slam = quat_slam_bak ;
quat_cam_slam = quatconj(quat_slam_bak);
pos_cam_slam = data_slam(:, 2:4); % cam0 not body
% for inav
quat_cam_inav = quatconj(data_inav(:, 5:8)); % body(imu) not cam0
pos_cam_inav = data_inav(:, 2:4); % cam0 not body

%% test function version
[Rrb_slam, pos_Ref] = slam2groundtruth( quat_cam_slam, pos_cam_slam, quat_gd, pos_gd);
[Rrb_inav, pos_Ref_inav] = slam2groundtruth( quat_cam_inav, pos_cam_inav, quat_gd, pos_gd);

%% attitude analysis function version
quat_rb_slam = dcm2quat(Rrb_slam);
quat_diff = attitude_diff(quat_rb_slam, quat_gd);
[anz_gd, any_gd, anx_gd] = quat2angle(quatconj(quat_gd));
[anz_slam, any_slam, anx_slam] = quat2angle(quatnormalize(quat_cam_slam));
[anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatnormalize(quat_rb_slam));
% euler angle difference of ground truth and slam
% [anz_diff, any_diff, anx_diff] = quat2angle(quat_diff); % or get this directly from anz_gd - anz_rb_slam

%% 

%% line2show 1, 2, 3
line2show = 3;
figure;
plot(data_slam(:,1), pos_Ref(:,line2show), 'b'); hold on
plot(data_inav(:,1), pos_Ref_inav(:,line2show), 'g');
plot(data_slam(:,1), pos_gd(:,line2show), 'r'); hold off
legend('body from slam', 'body from inav','body from ground truth')
title('zaxis comparing whole data')

%% local plotting
% inav start timestamp in data_slam
% st_gtruth = data_gtruth(1,1) * 1.0e-9;
% 	1. V103_FT_IMU01_05.txt 1403715930.0==>1403715935.0;
% 	2. V103_FT_IMU02_05.txt	1403715950.0==>1403715955.0;
% 	3. V103_FT_IMU03_05.txt 1403715970.0==>1403715975.0;
% vst_tstamp = [1403715930.0; 1403715950.0; 1403715970.0];
vend_tstamp = [];
% data_id = 1;
st_tstamp_inav = vst_tstamp(data_id) ;
end_tstamp_inav = vst_tstamp(data_id) + duration;
[tdiff0_slam, idx0_slam] = min( abs(data_slam(:,1) - st_tstamp_inav));
[tdiff1_slam, idx1_slam] = min( abs( data_slam(:,1) - end_tstamp_inav ));
[tdiff0_inav, idx0_inav] = min( abs(data_inav(:,1) - st_tstamp_inav));
[tdiff1_inav, idx1_inav] = min( abs( data_inav(:,1) - end_tstamp_inav ));
[tdiff0_gtruth, idx0_gtruth] = min( abs( data_gtruth4slam(:,1) - st_tstamp_inav));
[tdiff1_gtruth, idx1_gtruth] = min( abs( data_gtruth4slam(:,1) - end_tstamp_inav ));
%% slam, inav, ground truth
line2show = 1;
figure;
plot(data_slam(idx0_slam:idx1_slam, 1), pos_Ref(idx0_slam:idx1_slam, line2show), 'b'); hold on
plot(data_inav(idx0_inav : idx1_inav, 1), pos_Ref_inav(idx0_inav : idx1_inav, line2show), 'g');
plot(data_slam(idx0_slam:idx1_slam, 1), pos_gd(idx0_slam:idx1_slam, line2show), 'r'); hold off
legend('body from slam', 'body from inav','body from ground truth')
title('local plotting xaxis')
%% only slam and inav
line2show = 1;
trans_inav = pos_Ref(idx0_slam, :) - pos_Ref_inav(idx0_inav, :);
figure;
plot(data_slam(idx0_slam:idx1_slam, 1), pos_Ref(idx0_slam:idx1_slam, line2show), 'b'); hold on
plot(data_inav(idx0_inav : idx1_inav, 1), pos_Ref_inav(idx0_inav : idx1_inav, line2show) + ...
    trans_inav(line2show), 'g');
hold off
legend('body from slam', 'body from inav')
title('xaxis body position local plotting')
%% slam and inav difference
pos_diff_slam_inav =  pos_Ref(idx0_slam:idx1_slam, :) -  pos_Ref_inav(idx0_inav : idx1_inav, :);
% pos_diff_slam_inav = pos_diff_slam_inav - trans_inav;
figure;
plot(data_inav(idx0_inav : idx1_inav, 1), pos_diff_slam_inav(:,1), 'b');hold on
plot(data_inav(idx0_inav : idx1_inav, 1), pos_diff_slam_inav(:,2), 'g');
plot(data_inav(idx0_inav : idx1_inav, 1), pos_diff_slam_inav(:,3), 'r');hold off
legend('x error', 'y error', 'z error')
title('error local plotting')
%% transformation procedure
% figure;
% plot(data_slam(:,1), pos_cam_slam(:,line2show)); hold on
% plot(data_slam(:,1), pos_body_slam(:,line2show), 'r');

%% attitude plotting
att_viorb_plot
%% plotting attitude comparing
figure
subplot(3,1,1)
plot(data_gtruth4slam(:,1), anz_gd(:), 'r')
hold on
plot(data_slam(:,1), anz_rb_slam, 'b')
title('anz gd and slam')
subplot(3,1,2)
plot(data_gtruth4slam(:,1), any_gd(:), 'r')
hold on
plot(data_slam(:,1), any_rb_slam, 'b')
title('any gd and slam')
subplot(3,1,3)
plot(data_gtruth4slam(:,1), anx_gd(:), 'r')
hold on
plot(data_slam(:,1), anx_rb_slam, 'b')
title('anx gd and slam')
hold off

%% plotting attitude diff
anz_diff = anz_gd(:) - anz_rb_slam;
any_diff = any_gd(:) - any_rb_slam;
anx_diff = anx_gd(:) - anx_rb_slam;
figure
subplot(3,1,1)
plot(data_gtruth4slam(:,1), anz_diff)
title('anz diff gd and slam')
subplot(3,1,2)
plot(data_gtruth4slam(:,1), any_diff)
title('any diff gd and slam')
subplot(3,1,3)
plot(data_gtruth4slam(:,1), anx_diff)
title('anx diff gd and slam')
hold off
suptitle('slam attitude error to ground truth')
%% diff statistics
% figure
% subplot(3,1,1)
% histogram(anz_diff)
% subplot(3,1,2)
% histogram(any_diff)
% subplot(3,1,3)
% histogram(anx_diff)

%%
[pos_slam, quat_slam] = body_in_slam(pos_cam_slam, quat_cam_slam);% get body(imu) position from cam0 position
slam2RefFrame
