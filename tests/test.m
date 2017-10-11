%% quatrotate
q = [1 0 1 0];
r = [1 1 1];
n = quatrotate(q, r)
Rdcm = quat2dcm(q);
ndcm = Rdcm * r'
Rrotm = quat2rotm(q);
nrotm = Rrotm * r'

%% direction cosine matrix
% rotate the body frame around z axis 90 degree, and what's the direction
% cosine matrix C, such that p_g = C * p_b ?
% C = [0 -1 0; 1 0 0;0 0 1];
C = [0 -1 0; 1 0 0;0 0 1];
q90 = dcm2quat(C);
qq90 = rotm2quat(C);

%% quaternion test
% generate a quaternion

%% quaternion
% doc Quaternion Rotation, rotate vector by quaternion.

%%
axang = [ 1 0 0 pi/2;0 1 0 pi/2;0 0 1 pi/2];
quat = axang2quat(axang);
%% quat2rotm
%% hamilton or jpl
% JPL, left hand sys, see "Indirect kalman filter for 3d attitude
% estimation" from Trawny
% p_l = C_{lg} * p_g, transform global p_g to local p_l, using direction
% cosine matrix C_{lg}
% C_{lg} = (2q4^2 - 1)I - 2q4*skew(qimg) + 2 qimg *qimg^T
% 
% Hamilton, see wiki rotation matrix
% Q = [1 - 2y^2 -2z^2, 2xy - 2zw, 2xz + 2yw;
%      2xy + 2zw, 1 - 2x^2-2z^2, 2yz-2xw;
%      2xz-2yw, 2yz+2xw, 1-2x^2-2y^2];
rotmat = quat2rotm(quat);
% quati = quat_gen(axang);
function quats = quat_gen(axang)
    ang = axang(4)/pi*90;
    quats = [cosd(ang), sind(ang)*axang(1:3)];
end

%% global and local conversions
% quaternions
% rotation matrix
% position, velocity, translation
