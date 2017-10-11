%% dcm approach.
%% prepare the data
% pos_gd, quat_gd, quat_slam, pos_slam

R_rb = quat2dcm(quat_gd);
R_wb = quat2dcm(quat_slam);
R_rb0 = quat2dcm(quat_gd(1, :));
pos_rb0 = pos_gd(1,:)';
% pos_rbo = [];pos_gd()
% slam initialize with [0,0,0, 1, 0,0,0], px,py,pz, qw, qx,qy,qz
R_wb0 = quat2dcm(quat_slam(1,:)); % should be eye(3)
pos_wb0 = pos_slam(1,:)'; % should be [0,0,0]
%% slam world origin in Reference Frame
R_rwo = R_rb0 * R_wb0';% this should equals ground truth at slam ini timestamp
% R_rwo == R_rb0; % 
pos_rwo = pos_rb0 + (-R_rwo * pos_wb0);
% pos_rwo = pos_rbo;% should be the same with pos_rb0.
%% position in Ref from slam
% we can use both pos_rwo and pos_rb0 to get pos_rb_slam.
pos_rb_slam = pos_rwo + R_rwo * pos_slam';
pos_rb_slam = pos_rb_slam';
% or using pos_rb0 to calc pos_rb_slam, 
% pos_rb_slam = pos_rb0 + R_rwo*(pos_slam - pos_wb0);
%% attitude in Ref from slam
R_rb_slam = zeros(size(R_wb));
Rdiff = zeros(size(R_wb));
for i=1:size(R_wb,3)
    tmp_R_rb = R_rwo * R_wb(:,:,i);
    Rdiff(:, :, i) = R_rb(:,:, i)' * tmp_R_rb;% bug!
    % some checks later
    R_rb_slam(:,:,i) = tmp_R_rb;
end

quat_diff = dcm2quat(Rdiff);
quat_rb_slam = dcm2quat(R_rb_slam);
[anz_gd, any_gd, anx_gd] = quat2angle(quatnormalize(quat_gd));
[anz_slam, any_slam, anx_slam] = quat2angle(quatnormalize(quat_slam));
[anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatnormalize(quat_rb_slam));
%% quat_diff
% axangle_qdiff = quat2axang(quat_diff);
% figure; plot(data_slam(:,1), axangle_qdiff(:,4))

%% quat approach