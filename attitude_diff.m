function quat_diff = attitude_diff(quat_rb_slam, quat_gd)
%% attitude analysis
% quat_rb_slam = dcm2quat(Rrb_slam);
% Rrb_gd = quat2dcm( quat_gd );
% Rdiff = zeros(size(Rrb_slam)); 
quat_diff = zeros(size(quat_rb_slam));
for idx = 1 : length(quat_rb_slam)
%    Rdiff(:,:,idx) = Rrb_gd(:, :, idx) * Rrb_slam(:, :, idx)';
    quat_diff(idx,:) = quatdivide(quat_rb_slam(idx, :), quatconj(quat_gd(idx, :)));
end

end