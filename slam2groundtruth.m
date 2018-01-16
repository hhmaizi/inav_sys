function [Rrb_slam, pos_Ref] = slam2groundtruth( quat_cam_slam, pos_cam_slam,quat_gd, pos_gd)
%
Tbc = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0];
%%
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

%% attitude alignment
% quat_rb_slam = dcm2quat(Rrb_slam);
% Rrb_gd = quat2dcm( quat_gd );
% Rdiff = zeros(size(Rrb_slam)); quat_diff = zeros(size(quat_rb_slam));
% for idx = 1 : length(Rrb_slam)
%     Rdiff(:,:,idx) = Rrb_gd(:, :, idx) * Rrb_slam(:, :, idx)';
%     quat_diff(idx,:) = quatdivide(quat_rb_slam(idx, :), quatconj(quat_gd(idx, :)));
% end
%% attitude conversion
% quat_diff = dcm2quat(Rdiff);
% % Rdiff_quat = quat2dcm(quat_diff);
% % [anz_gd, any_gd, anx_gd] = quat2angle(quatconj(quat_gd));
% % [anz_slam, any_slam, anx_slam] = quat2angle(quatnormalize(quat_cam_slam));
% % [anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatnormalize(quat_rb_slam));
% euler angle difference of ground truth and slam
% % [anz_diff, any_diff, anx_diff] = quat2angle(quat_diff);
% figure; plot(anz_diff, 'r');hold on
% plot(any_diff, 'g');plot(anx_diff, 'b');
end