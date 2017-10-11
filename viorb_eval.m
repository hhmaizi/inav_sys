%% Main Postprocessing routine
% 09.26.2017
% author: Ted Zhang, Pico
% this script is modified from Julian Surber, ETH Zurich
% see https://github.com/surberj/Vision for more info.
%
% This file loads data from a csv file of a ros session to analyse the
% performance of a SLAM algorithm like ROVIO, PTAM, OKVIS, ...
%
% The csv file must provide a header row describing the different signals
% 
% The following signals must be provided by the csv file
%       timestamp
%       

% add subfolders to matlab path
clear;
% close all
clc
file=which('viorb_eval.m');
[scriptPath] = fileparts(file);
cd(scriptPath);
addpath(genpath(scriptPath))

%% load data
%filepath = '\\10.10.194.34\foaa\SLAM\DATASET\WEB_DATA\EUROC_DATASET_1\V1_03_difficult\state_groundtruth_estimate0\';
filepath = '\\10.10.194.34\foaa\SLAM\DATASET\WEB_DATA\EUROC_DATASET_1\MH_03_medium\mav0\state_groundtruth_estimate0\';
%filename = uigetfile([filepath, '*.csv'],'Select the .csv file containing ground_truth pose');
% using summary(table_name) to get the table info summary.
%gtruth = readtable([filepath, filename]);
gtruth = readtable([filepath, 'data.csv']);
%
% filepath = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\';
% slam = readtable([filepath, 'InavTrajectory.csv']);
% filename = uigetfile([filepath, '*.csv'],'Select the .csv file containing the inav trajectory.');
% slam = readtable([filepath, filename]);

filepath = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\';
slam = readtable([filepath, 'mh03_vio_cam.csv']);
% convert and time-align

%% quaternion to euler convertion

% ground_truth, perhaps eth is using JPL.
% so I changed back using q =[w, x, y, z];
% prism marker attitude in vicon.
quat_gtruth_vicon = [gtruth.q_RS_w__, gtruth.q_RS_x__, gtruth.q_RS_y__, gtruth.q_RS_z__];
% body(imu) position in vicon, grund truth has been aligned, see ground
% truth for more info. not prisma position. however we can check this with
% prisma position and T_{BS} from vicon folder.
pb_gtruth_vicon = [gtruth.p_RS_R_x_m_, gtruth.p_RS_R_y_m_, gtruth.p_RS_R_z_m_];

% p_vicon = [gtruth.p_RS_R_x_m_, gtruth.p_RS_R_y_m_, ground_truth.p_RS_R_z_m_];

% viorb position and attitude in world(expressed in world)
% be clear about world, is it the same world with euroc, or is it the world
% for the initialized slam world?
% Camera.Tbc:
%  [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
%   0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
%   -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
%   0.0, 0.0, 0.0, 1.0]

Tbc = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;...
       0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;....
       -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;...
       0.0, 0.0, 0.0, 1.0];
Rbc = Tbc(1:3,1:3); bc_b = Tbc(1:3,4);
% get quatb_slam
quatc_slam = [slam.qw_, slam.qx_, slam.qy_, slam.qz_];
Rsc = quat2dcm(quatc_slam); Rsb = zeros(size(Rsc));
for i = 1: size(Rsc,3)
    Rsb(:,:,i) = Rsc(:,:,i) * Rbc';
end

quatb_slam = dcm2quat(Rsb);
% get pb_slam
pc_slam = [slam.px_, slam.py_, slam.pz_];% this is pc_slam
% using Tbc to convert pc_slam to pb_slam
% pb_slam = pc_slam + cb_slam;
% R_sc = quat2rot( q_cam );
% cb_slam = R_sc * cb_cam
% cb_cam = -Rbc^T * bc_b;
% Tbc = [Rbc, bc_b; 01x3, 1];
cb_cam = -Rbc' * bc_b; cb_slam = Rsc(:,:,1) * cb_cam;
pb_slam = [pc_slam(:,1) + cb_slam(1), ...
           pc_slam(:,2) + cb_slam(2), ...
           pc_slam(:,3) + cb_slam(3)];

%% rotation matrix between rovio and vicon
% align time stamp between ground truth and viorb.
% time vectors, units nano seconds  since 1970(unix posix)
time_gtruth = gtruth.x_timestamp;
time_slam = slam.x_timestamp_*1.0e9;
if time_gtruth(1) < time_slam(1)
    % k = find(X,n) returns the first n indices corresponding to the nonzero elements in X.
    ind_align_gtruth = find(time_gtruth >= time_slam(1),1);
    ind_align_slam = 1;
else
    ind_align_gtruth = 1;
    ind_align_slam = find(time_slam >= time_gtruth(1),1);
end
%%
% direction cosine matrix calculated from inner product, so it won't change
% whether you choose which frame to express the axis vecs, as long as you
% choose the same frame for both slam and vicon.
R_vb = quat2dcm(quat_gtruth_vicon(ind_align_gtruth,:));% body in vicon world.
R_sb = quat2dcm(quatb_slam(ind_align_slam,:));% body to vio world rot, not leica or vicon
pb_aligned_gtruth_vicon = pb_gtruth_vicon(ind_align_gtruth,:)';% vec from vicon to body(imu) in vicon
pb_aligned_slam = pb_slam(ind_align_slam,:)';% vec from slam to body(imu) in slam(viorb)

% constant rotation and translation between vicon and slam world frame
R_vs = R_vb * R_sb'; % R_vs: slam to vicon.
vs_vicon = pb_aligned_gtruth_vicon - R_vs * pb_aligned_slam;

%% check, nonsense check!
if norm(vs_vicon - pb_aligned_gtruth_vicon + R_vs * pb_aligned_slam) > 0.01
    print('there seems to be something wrong with the rotations')
end

% describe all signals wrt vicon frame
% viorb: position:    
%       vec_vb_in_vicon_viorb = vec_vs_in_vicon + R_vs * vec_sb_in_viorb
%       orientation: 
%       ang_vb_in_vicon_rovio = dcm2angle(R_vr * R_rb)
% vicon: position:
%        r_vb_in_vicon_vicon = r_vicon
%        orientation: 
%        ang_vb_in_vicon_vicon = dcm3angle(R_vb)
%%
vb_vicon_slam = zeros(length(pb_slam),3);
ang_vb_vicon_slam = zeros(length(pb_slam),3);
for i=1:length(pb_slam)
    vb_vicon_slam(i,:) = vs_vicon + R_vs*pb_slam(i,:)';
    R_rb_tmp = quat2dcm(quatb_slam(i,:));
    ang_vb_vicon_slam(i,:) = dcm2angle(R_vs*R_rb_tmp);
end

pb_vicon_vicon = pb_gtruth_vicon; % or use pb_gtruth_vicon
ang_vb_vicon_vicon = zeros(length(pb_gtruth_vicon),3);
[ang_vb_vicon_vicon(:,1),...
    ang_vb_vicon_vicon(:,2),...
    ang_vb_vicon_vicon(:,3)] = quat2angle(quat_gtruth_vicon);

%% plot
%
% init figure
scrsz = get(groot,'ScreenSize');
figure('Name','Viorb analysis','NumberTitle','off', ...
    'Position',[1 1 scrsz(3) scrsz(4)]);
% 
ha(1) = subplot(321);
plot(time_gtruth, pb_vicon_vicon(:,1))
hold on
plot(time_slam, vb_vicon_slam(:,1))
title('x tracking','FontSize',12)
h_legend = legend('ground truth', 'inav');
set(h_legend,'FontSize',8)
xlabel('[s]','FontSize',12)
ylabel('[m]','FontSize',12)
grid on

ha(2) = subplot(323);
plot(time_gtruth,pb_vicon_vicon(:,2))
hold on
plot(time_slam,vb_vicon_slam(:,2))
title('y tracking','FontSize',12)
h_legend = legend('ground truth', 'inav');
set(h_legend,'FontSize',8)
xlabel('[s]','FontSize',12)
ylabel('[m]','FontSize',12)
grid on

ha(3) = subplot(325);
plot(time_gtruth,pb_vicon_vicon(:,3))
hold on
plot(time_slam,vb_vicon_slam(:,3))
title('z tracking','FontSize',12)
h_legend = legend('ground truth', 'inav');
set(h_legend,'FontSize',8)
xlabel('[s]','FontSize',12)
ylabel('[m]','FontSize',12)
grid on

% attitude
ha(4) = subplot(322);
plot(time_gtruth, ang_vb_vicon_vicon(:,1))
hold on
plot(time_slam, ang_vb_vicon_slam(:,1))
title('euler r1 tracking','FontSize',12)
h_legend = legend('ground truth', 'slam');
set(h_legend,'FontSize',8)
xlabel('[s]','FontSize',12)
ylabel('[rad]','FontSize',12)
grid on

ha(4) = subplot(324);
plot(time_gtruth, ang_vb_vicon_vicon(:,2))
hold on
plot(time_slam,ang_vb_vicon_slam(:,2))
title('euler r2 tracking','FontSize',12)
h_legend = legend('ground truth', 'rovio');
set(h_legend,'FontSize',8)
xlabel('[s]','FontSize',12)
ylabel('[rad]','FontSize',12)
grid on

ha(4) = subplot(326);
plot(time_gtruth, ang_vb_vicon_vicon(:,3))
hold on
plot(time_slam, ang_vb_vicon_slam(:,3))
title('euler r3 tracking', 'FontSize',12)
h_legend = legend('ground truth', 'slam');
set(h_legend, 'FontSize',8)
xlabel('[s]', 'FontSize',12)
ylabel('[rad]', 'FontSize',12)
grid on


linkaxes(ha,'x')