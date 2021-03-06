%clc
clear all
close all
%% read the data from script
% viorb_readdata;
%% params: cam frame to imu frame transformation
Tbc = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0];

%% read the data directly
path_data = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\test0929\'; % #timestamp p_RS_R_x  p_RS_R_y p_RS_R_z q_RS_w q_RS_x q_RS_y q_RS_z v_RS_R_x v_RS_R_y v_RS_R_z
filenames_slam = {'M01_FT_VI.txt', 'M03_FT_VI.txt', 'V103_FT_VI.txt', 'V203_FT_VI.txt'};

data_gtruth = load('mh01_groundtruth.csv');
% data_slam = load([path_data, filenames_slam{1}]);
data_slam = load('.\data\mh01.txt');
% find data_imu index where imu track only starts.

%% find corresponding ground truth timestamp to data_slam 
% data_slam's timestamp should starts somewhere from ground truth.
% check timestamp, ground truth ends before slam.
% manually make slam ends before ground truth. thresh hold 3.0e+09 nano sec
data_slam = data_slam(1: end - 3*20,:); % cut 3 seconds data from slam, assuming slam frequency 20hz
denom_timestamp = 1.0e+09 * (60 * 60 * 24 * 365); % nano secs per year.
yearsince1972 = data_gtruth(1,1)/denom_timestamp;
% slam timestamp and groundtruth timestamp shouldn't diff more than 1 year.
% maybe one hour for later implementation.
if (data_slam(1,1)/denom_timestamp < yearsince1972 - 1 || ...
       data_slam(1,1)/denom_timestamp > yearsince1972 + 1 )
   disp 'check your data timestamp'
   disp 'unit nano sec, since 1972'
   return
end
if (((data_slam(1,1) - data_gtruth(1,1)) < 0)  || ...
        ((data_slam(end,1) - data_gtruth(end,1)) > 0) )
    disp 'slam data starts out of ground truth timestamps.'
    return
elseif ( (data_slam(1,1) - data_gtruth(1,1) > 300.0e+09))
    disp 'perhaps data_slam and data_gtruth did not match.'
    disp 'check your data.'
    return
end

% time alignment
idx_gtruth4slam = [];
time_diff_gtruth4slam = [];
idx_bad4slam = [];
time_diff_bad4slam = [];
for t_slam = data_slam(:,1)' % always row vec for for statement
    [t_diff, idx_tmp] = min(abs(data_gtruth(:,1) - t_slam));
    if t_diff > 0.02e+09 % slam freq 20hz(0.05s)
        idx_bad4slam = [idx_bad4slam; idx_tmp];
        time_diff_bad4slam = [time_diff_bad4slam; t_diff];
        break;
    end
    idx_gtruth4slam = [idx_gtruth4slam; idx_tmp];
    time_diff_gtruth4slam = [time_diff_gtruth4slam; t_diff];
end
if (size(idx_gtruth4slam,1) ~= size(data_slam,1))
    disp 'something wrong with time alignment between ground truth and data slam.'    
    disp 'check your program.'
    return
end
%
data_gtruth4slam = data_gtruth(idx_gtruth4slam, 1:8);
% adjust timestamp, so it starts from 0 sec
data_slam(:,1) = (data_slam(:,1) - data_slam(1,1)) * 1.0e-09;
data_gtruth4slam(:,1) = (data_gtruth4slam(:,1) - data_gtruth4slam(1,1)) * 1.0e-09;
% data_gtruth4slam(1,1) - data_gtruth4slam(end,1)

%% transform slam to Reference frame
% preparing the raw data
quat_gd = data_gtruth4slam(:, 5:8);
pos_gd = data_gtruth4slam(:,2:4);
quat_slam_bak = data_slam(:, 5:8); % body(imu) not cam0
% quat_cam_slam = quat_slam_bak ;
quat_cam_slam = quatconj(quat_slam_bak);
pos_cam_slam = data_slam(:, 2:4); % cam0 not body

%% align slam to ground truth
Rsc = quat2dcm(quat_cam_slam);
Rrb_slam = zeros(size(Rsc));
vcb_slam = zeros(size(pos_cam_slam));
for idx = 1:length(pos_cam_slam)
    vcb_slam(idx,:) = ( Rsc(:,:,idx) * (-Tbc(1:3, 1:3)' * Tbc(1:3,4)) )';
    Rrb_slam(:, :, idx) = quat2dcm(quatconj(quat_gd(1,:))) * Tbc(1:3, 1:3) * Rsc(:,:,idx) * Tbc(1:3, 1:3)';    
end
pos_body_slam = pos_cam_slam + vcb_slam;
pos_body_B0 = Tbc(1:3,1:3) * pos_body_slam' + Tbc(1:3,4);
% pos_Ref = quat2dcm(quat_gd(1,:)) * pos_body_B0 + pos_gd(1,:)';
pos_Ref = quat2dcm(quatconj(quat_gd(1,:))) * pos_body_B0 + pos_gd(1,:)';
pos_Ref = pos_Ref';

%% body position comparing. slam VS ground truth.  
% line2show 1, 2, 3
line2show = 1;
figure;
plot(data_slam(:,1), pos_Ref(:,line2show)); hold on
plot(data_slam(:,1), pos_gd(:,line2show), 'r'); hold off
title('body position error--slam to ground truth')
legend('body from slam', 'body from ground truth')
%% transformation procedure
% figure;
% plot(data_slam(:,1), pos_cam_slam(:,line2show)); hold on
% plot(data_slam(:,1), pos_body_slam(:,line2show), 'r');
%% attitude alignment
quat_rb_slam = dcm2quat(Rrb_slam);
Rrb_gd = quat2dcm( quat_gd );
Rdiff = zeros(size(Rrb_slam)); quat_diff = zeros(size(quat_rb_slam));
for idx = 1 : length(Rrb_slam)
    Rdiff(:,:,idx) = Rrb_gd(:, :, idx) * Rrb_slam(:, :, idx)';
    quat_diff(idx,:) = quatdivide(quat_rb_slam(idx, :), quatconj(quat_gd(idx, :)));
end
% attitude conversion
% quat_diff = dcm2quat(Rdiff);
Rdiff_quat = quat2dcm(quat_diff);
[anz_gd, any_gd, anx_gd] = quat2angle(quatconj(quat_gd));
[anz_slam, any_slam, anx_slam] = quat2angle(quatnormalize(quat_cam_slam));
[anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatnormalize(quat_rb_slam));

%% attitude from slam and ground truth
figure
subplot(3,1,1)
plot(data_gtruth4slam(:,1), anz_gd(:), 'r')
hold on
plot(data_gtruth4slam(:,1), any_gd(:), 'g')
plot(data_gtruth4slam(:,1), anx_gd(:), 'b')
title('body ground truth Ref')
subplot(3,1,2)
plot(data_slam(:,1), anz_rb_slam, 'r')
hold on
plot(data_slam(:,1), any_rb_slam, 'g')
plot(data_slam(:,1), anx_rb_slam, 'b')
title('slam body aligned to Ref')
subplot(3,1,3)
plot(data_slam(:,1), anz_slam, 'r')
hold on
plot(data_slam(:,1), any_slam, 'g')
plot(data_slam(:,1), anx_slam, 'b')
title('slam cam unaligned')
hold off
suptitle('attitudes ground truth, slam body, and slam cam')
% comparing attitude between slam's body and ground truth.
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
%% attitude error plotting
% anz_diff = anz_gd(:) - anz_rb_slam;
% any_diff = any_gd(:) - any_rb_slam;
% anx_diff = anx_gd(:) - anx_rb_slam;
% figure
% subplot(3,1,1)
% plot(data_gtruth4slam(:,1), anz_diff)
% title('anz diff gd and slam')
% subplot(3,1,2)
% plot(data_gtruth4slam(:,1), any_diff)
% title('any diff gd and slam')
% subplot(3,1,3)
% plot(data_gtruth4slam(:,1), anx_diff)
% title('anx diff gd and slam')
% hold off
%
%% diff statistics
% figure
% subplot(3,1,1)
% histogram(anz_diff)
% subplot(3,1,2)
% histogram(any_diff)
% subplot(3,1,3)
% histogram(anx_diff)
