%% test for transform from body(imu) to vicon,
% T_BS from sensor.yaml in vicon folder
clear
close all

% load data
filepath = '\\10.10.194.34\foaa\SLAM\DATASET\WEB_DATA\EUROC_DATASET_1\V1_03_difficult\';
filename = uigetfile('\\10.10.194.34\foaa\SLAM\DATASET\WEB_DATA\EUROC_DATASET_1\V1_03_difficult\vicon0\*.csv', 'select the vicon data from vicon folder.');
vicon = readtable([ filepath,'vicon0\',filename]);
filename = uigetfile([filepath, 'state_groundtruth_estimate0\*.csv'], 'select csv data for imu');
imu = readtable([filepath, 'state_groundtruth_estimate0\', filename]);

%% quat and position from raw data
quat_vicon = [vicon.q_RS_w__, vicon.q_RS_x__, vicon.q_RS_y__, vicon.q_RS_z__];
pos_vicon_r = [vicon.p_RS_R_x_m_, vicon.p_RS_R_y_m_, vicon.p_RS_R_z_m_];
quat_imu = [imu.q_RS_w__, imu.q_RS_x__, imu.q_RS_y__, imu.q_RS_z__];% it's ground truth, aligned with imu.
pos_imu_r = [imu.p_RS_R_x_m_, imu.p_RS_R_y_m_, imu.p_RS_R_z_m_]; % ground truth aligned with imu.
% time alignment
time_vicon = vicon.x_timestamp_ns_;
time_imu = imu.x_timestamp;
if time_vicon(1) < time_imu(1)
    ind_align_vicon = find(time_vicon >= time_imu(1),1);
    ind_align_imu = 1;
else
    ind_align_vicon = 1;
    ind_align_imu = find(time_imu >= time_vicon(1),1);
end
%% plotting
xshift = pos_imu_r(1,1) - pos_vicon_r(ind_align_vicon, 1);
yshift = pos_imu_r(1,2) - pos_vicon_r(ind_align_vicon, 2);
zshift = pos_imu_r(1,3) - pos_vicon_r(ind_align_vicon, 3);
figure;
subplot(3,1,1)
plot(time_imu, pos_imu_r(:,1))
hold on;plot(time_vicon, pos_vicon_r(:,1), 'r')
hold off

subplot(3,1,2)
plot(time_imu, pos_imu_r(:,2))
hold on;plot(time_vicon, pos_vicon_r(:,2), 'r')

subplot(3,1,3)
plot(time_imu, pos_imu_r(:,3))
hold on;plot(time_vicon, pos_vicon_r(:,3), 'r')
title('raw data compare vicon with ground truth')
legend('ground truth', 'vicon')

%%
% T_BV from vicon
T_bv = [ 0.33638, -0.01749,  0.94156,  0.06901;...
         -0.02078, -0.99972, -0.01114, -0.02781;...
          0.94150, -0.01582, -0.33665, -0.12395;...
              0.0,      0.0,      0.0,      1.0];

% infostr = ['ground truth timestamp - vicon timestamp',  num2str()];
fprintf('ground truth timestamp - vicon timestamp = %f nano seconds.\n', time_imu(ind_align_imu) - time_vicon(ind_align_vicon));
% T_rb from attitude of imu
% Trb = [ Rrb, rRB'; 0, 1];
R_rb = quat2dcm(quat_imu(ind_align_imu,:));
rRB = pos_imu_r(ind_align_imu,:);
T_rb = [R_rb, rRB'; zeros(1,3), 1];

% get rRV from imu data.
imuv_r = R_rb * T_bv(1:3, 4);% imu to vicon vec in r
% time consuming
% pos_vicon_r_imu = pos_imu_r + repmat(imuv_r', size(pos_imu_r,1)); % position of vicon in R from imu data
pos_vicon_r_imu = [pos_imu_r(:, 1) + imuv_r(1), ...
                   pos_imu_r(:, 2) + imuv_r(2), ...
                   pos_imu_r(:, 3) + imuv_r(3)];

%% check the results
if (norm(pos_vicon_r_imu(1,:) - pos_vicon_r(ind_align_vicon,:)) > 0.002)
    disp 'something wrong happened, check you sys.'
else
    disp 'congratulatious!'
end

%% plotting
% xshift = pos_vicon_r_imu(1,1) - pos_vicon_r(ind_align_vicon, 1);
% yshift = pos_vicon_r_imu(1,2) - pos_vicon_r(ind_align_vicon, 2);
% zshift = pos_vicon_r_imu(1,3) - pos_vicon_r(ind_align_vicon, 3);
% time_imu_ = time_imu(ind_align_imu : end);
figure;
subplot(3,1,1)
plot(time_imu, pos_vicon_r_imu(:,1))
hold on;plot(time_vicon, pos_vicon_r(:,1), 'r')
hold off

subplot(3,1,2)
plot(time_imu, pos_vicon_r_imu(:,2))
hold on;plot(time_vicon, pos_vicon_r(:,2), 'r')

subplot(3,1,3)
plot(time_imu, pos_vicon_r_imu(:,3))
hold on;plot(time_vicon, pos_vicon_r(:,3), 'r')
title('aligned data')
%%
% figure;
% subplot(3,1,1)
% plot(time_imu, pos_vicon_r_imu(:,1))
% hold on;plot(time_vicon, pos_vicon_r(:,1) + xshift, 'r')
% hold off
% 
% subplot(3,1,2)
% plot(time_imu, pos_vicon_r_imu(:,2))
% hold on;plot(time_vicon, pos_vicon_r(:,2) + yshift, 'r')
% 
% subplot(3,1,3)
% plot(time_imu, pos_vicon_r_imu(:,3))
% hold on;plot(time_vicon, pos_vicon_r(:,3) + zshift, 'r')