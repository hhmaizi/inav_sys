% %% transform slam to Reference frame
% % preparing the raw data
% quat_gd = data_gtruth4slam(:, 5:8);
% pos_gd = data_gtruth4slam(:,2:4);
% quat_slam_bak = data_slam(:, 5:8); % body(imu) not cam0
% quat_cam_slam = quat_slam_bak ;
% % quat_cam_slam = quatconj(quat_slam_bak);
% pos_cam_slam = data_slam(:, 2:4); % cam0 not body

%% align slam to ground truth
Rcb = Tbc(1:3, 1:3)'; Rbc = Tbc(1:3, 1:3);% rigid, so Rb0c0 = Rbc
Rrb0 = quat2dcm(quatconj(quat_gd(1,:))); Rrb_gd = quat2dcm(quatconj(quat_gd));
% Rrb = Rrb0 * Rb0c0 * Rsc * Rcb
Rsc = quat2dcm(quat_cam_slam);
Rrb_slam = zeros(size(Rsc));
for idx = 1:length(pos_cam_slam)
    Rrb_slam(:, :, idx) = Rrb0 * Rbc' * Rsc(:,:,idx) * Rcb';    
end

%% attitude alignment
quat_rb_slam = dcm2quat(Rrb_slam);
Rdiff = zeros(size(Rrb_slam)); quat_diff = zeros(size(quat_rb_slam));
for idx = 1 : length(Rrb_slam)
    Rdiff(:,:,idx) = Rrb_gd(:, :, idx) * Rrb_slam(:, :, idx)';
    quat_diff(idx,:) = quatdivide(quat_rb_slam(idx, :), quatconj(quat_gd(idx, :)));
end
%% attitude conversion
% quat_diff = dcm2quat(Rdiff);
Rdiff_quat = quat2dcm(quat_diff);
[anz_gd, any_gd, anx_gd] = quat2angle(quatconj(quat_gd));
% [anz_gd, any_gd, anx_gd] = quat2angle(quat_gd);
[anz_slam, any_slam, anx_slam] = quat2angle(quatnormalize(quat_cam_slam));
[anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatnormalize(quat_rb_slam));
% [anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatconj(quat_rb_slam));
%% 
figure;
plot(anz_gd, 'r'); hold on
plot(anz_rb_slam);hold off
legend('ground truth', 'slam')
title('euler angle z compare')
%%
figure;
plot(any_gd, 'r'); hold on
plot(any_rb_slam);hold off
legend('ground truth', 'slam')
title('euler angle y compare')
%%
figure;
plot(anx_gd, 'r'); hold on
plot(anx_rb_slam);hold off
legend('ground truth', 'slam')
title('euler angle x compare')
%%
[anz_diff, any_diff, anx_diff] = quat2angle(quat_diff);
figure; plot(anz_diff, 'r');hold on
plot(any_diff, 'g');plot(anx_diff, 'b');
title('euler angle from quat_diff')
%%
%% plotting
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

%% transformation procedure
figure;
plot(data_slam(:,1), pos_cam_slam(:,line2show)); hold on
plot(data_slam(:,1), pos_slam(:,line2show), 'r');

%%
pos_body_slam = pos_cam_slam + vcb_slam;
pos_body_B0 = Tbc(1:3,1:3) * pos_body_slam' + Tbc(1:3,4);
% pos_Ref = quat2dcm(quat_gd(1,:)) * pos_body_B0 + pos_gd(1,:)';
pos_Ref = quat2dcm(quatconj(quat_gd(1,:))) * pos_body_B0 + pos_gd(1,:)';
pos_Ref = pos_Ref';
%%
[pos_slam, quat_slam] = body_in_slam(pos_cam_slam, quat_cam_slam);% get body(imu) position from cam0 position
slam2RefFrame

%% raw data plotting
% raw_plot

%% position plotting
pos_viorb_plot
%% attitude plotting
att_viorb_plot
%%
