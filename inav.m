%% inertial navigation using rk4 integration
% inputs:
%         qinit: input 4d array representing the initial rotation
%         omega0: initial angular velocity at time t0
%         omega1: initial angular velocity at time t1
%         dt: time step (t1 - t0)
% outputs:
%         qres: resulting final rotation
% for more info see "geometric integration of quaternions"

%% from local(body) to global transformation
% the quaternion approach
% to be implemented
% the direction matrix approach.
% The direction cosines representation the attitude of the body frame
% relative to the global frame is specified by a 3x3 rotation matrix C,
% in which each column is a unit vector alone one of the body axes
% specified in terms of the global axes.
% for more info see ch6.1, pp21 "an introduction to inertial navigation"
% from Oliver J. Woodman
% I need a test for direction cosine matrix
navState.p = zeros(3,1);
navState.v = zeros(3,1); % should I change this?
navState.q = [1, 0, 0, 0];
navState.a = []; % raw data
navState.omega = []; % raw data

%% acc local integration, later try global integration.
function [txl, tyl, tzl] = accIntLinear(acc, dt)
    
    [vx, vy, vz] = computeVel(acc, dt);
    txl = 0.5*dt*(vIni(1) + vx);
    tyl = 0.5*dt*(vIni(2) + vy);
    tzl = 0.5*dt*(vIni(3) + vz);
end

%% local linear velocity from acc
function [vxl, vyl, vzl] = computeVel(acc, dt)

    vl = vIni + acc * dt;
    vxl = vl(1); vyl = vl(2); vzl = vl(3);
end

%% rk4 quaternion integration 
function qres = quatIntRK4(qinit, omega0, omega1, dt)
    
    % average angular vel, for small angular vel.
    omega01 = 0.5 * (omega0 + omega1);
    
    omegaSkew = toSkewmat44(omega0);    
    k1 = 0.5 * omegaSkew * qinit;
    qtmp = qinit + 0.5*dt*k1;
    omegaSkew = toSkewmat44(omega01);    
    k2 = 0.5 * omegaSkew * qtmp;
    qtmp = qinit + 0.5*dt*k2;
    k3 = 0.5 * omegaSkew * qtmp;
    qtmp = qinit + dt*k3;
    omegaSkew = toSkewmat44(omega1);    
    k4 = 0.5 * omegaSkew * qtmp;
    % rk4 integration
    % 1/6 approx [0.166666666666667]
    qres = qinit + 0.166666666666667 * (k1 + 2.0*k2 + 2.0 * k3 + k4);
    qres = quatnormalize(qres);
end

%% convert angular vel to skew matrix
function omegaSkew = toSkewmat44(omega)
    omegaSkew = [0.0,      -omega(1), -omega(2), -omega(3);...
                 omega(1),  0.0,       omega(3), -omega(2);...
                 omega(2), -omega(3),  0.0,       omega(1);...
                 omega(3), omega(2),  -mega(1),  0.0];    
end

