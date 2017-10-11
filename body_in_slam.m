function [body_pos_slam, body_quat_slam] = body_in_slam(posc_slam, quatc_slam)
%Convert cam0's position to imu position
% pos_slam: cam0's position in slam
% quat_slam: 
% the following params is from T_BS
% vec_b = T_BS * vec_cam % transform a vec in cam to vec in body
%% checking inputs
if length(posc_slam) ~= length(quatc_slam)
    disp 'num of pos and num of slam did not equal.'
    return
end

%% calc Vcb_cam
% Vcb_cam = Tcb(1:3,4)
Pbc_b = [-0.0216;-0.0647;0.0098]; %
Tbc = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0];
     
Rcb = Tbc(1:3,1:3)';
% qcb = dcm2quat(Rcb);
% body_quat_slam = quatmultiply(quatc_slam, qcb);
Vcb_cam = -Rcb * Pbc_b;

%% calc Vcb_slam and Rsb
Rsb = zeros(3,3,length(quatc_slam));
Vcb_slam = zeros(size(posc_slam));
for idx = length(posc_slam)
    Rsc = quat2dcm(quatc_slam(idx,:));
    Rsb(:,:,idx) = Rsc * Rcb;
    Vcb_tmp = Rsc * Vcb_cam;
    Vcb_slam(idx,:) = Vcb_tmp';
end
%%
body_pos_slam = posc_slam + Vcb_slam;
body_quat_slam = dcm2quat(Rsb);
end

