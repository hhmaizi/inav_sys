%% dcm approach.
%% check data
% assuming quat_gd(1,:), pos_gd, corresponding to slam [0 0 0 1 0 0 0];
% if (norm(pos_slam(1,:) - [ 0 0 0]) ~= 0 ) &&...
%         (norm(quat_slam(1,:) - [1 0 0 0]))
%     disp 'seems your slam data did not starts from origin [0 0 0 1 0 0 0]'
%     return
% end
%% prepare the data
% pos_gd, quat_gd, quat_slam, pos_slam

%% world frame in Ref
Rrb0 = quat2dcm(quat_gd(1,:));
Rbc = Tbc(1:3,1:3);
Rrw = Rrb0 * Rbc;
pos_rb_slam = pos_gd(1,:)' + Rrb0 * (Tbc(1:3,4) + Rbc * pos_slam');
% Vcb_cam = -Rbc' * Tbc(1:3,4);
% Vbw_R = Rrw * Vcb_cam;
%
% Vrw_R = pos_gd(1,:) + Vbw_R';

%% body position in Ref frame from slam, Vrb_R = Vrw_R + Vwb_R
% body in Ref frame
% Vwb_R = Rrw * pos_slam';
% pos_rb_slam = Vrw_R' + Vwb_R;
pos_rb_slam = pos_rb_slam';

%% body attitude in Ref from slam
%
Rwb = quat2dcm(quat_slam);
Rrb_slam = zeros(size(Rwb));
Rdiff = zeros(size(Rrb_slam));
Rrb_gd = quat2dcm(quat_gd);
for idx = 1:length(Rwb)
    Rrb_slam(:,:,idx) = Rrw * Rwb(:,:,idx);
    Rdiff = Rrb_gd(:,:,idx)' * Rrb_slam(:,:,idx);
end
quat_rb_slam = dcm2quat(Rrb_slam);

%% 
quat_diff = dcm2quat(Rdiff);
[anz_gd, any_gd, anx_gd] = quat2angle(quatnormalize(quat_gd));
[anz_slam, any_slam, anx_slam] = quat2angle(quatnormalize(quat_slam));
[anz_rb_slam, any_rb_slam, anx_rb_slam] = quat2angle(quatnormalize(quat_rb_slam));
%% quat_diff
% axangle_qdiff = quat2axang(quat_diff);
% figure; plot(data_slam(:,1), axangle_qdiff(:,4))
% title('quat_diff')
%% quat approach