function [data_slam_taligned, gtruth_taligned2slam] = align_time_stamp(data_slam, data_gtruth)
%% find corresponding ground truth to data_slam, according to timestamp 
% assuming data_slam's timestamp starts somewhere from ground truth
% timestamp. 
% check timestamp, suppose input's timestamp's unit is nanosecond
% ground truth ends before slam.
% manually make slam ends before ground truth. thrshhold 3.0e+09 nano sec
data_slam_taligned = data_slam(1: end - 3*20,:); % assuming slam frequency 20hz
% 
denom_timestamp = 1.0e+09 * (60 * 60 * 24 * 365);
yearsince1972 = data_gtruth(1,1)/denom_timestamp;
%% check your data
if (data_slam_taligned(1,1)/denom_timestamp < yearsince1972 - 1 || ...
       data_slam_taligned(1,1)/denom_timestamp > yearsince1972 + 1 )
   disp 'check your data timestamp'
   disp 'unit nano sec, since 1972'
   return
end

if (((data_slam_taligned(1,1) - data_gtruth(1,1)) < 0)  || ...
        ((data_slam_taligned(end,1) - data_gtruth(end,1)) > 0) )
    disp 'slam data starts out of ground truth timestamps.'
    return
elseif ( (data_slam_taligned(1,1) - data_gtruth(1,1) > 300.0e+09))
    disp 'perhaps data_slam and data_gtruth did not match.'
    disp 'check your data.'
    return
end

%% align ground truth and slam timestamp.
idx_gtruth4slam = [];
time_diff_gtruth4slam = [];
idx_bad4slam = [];
time_diff_bad4slam = [];
for t_slam = data_slam_taligned(:,1)' % always row vec for for statement
    [t_diff, idx_tmp] = min(abs(data_gtruth(:,1) - t_slam));
    if t_diff > 0.02e+09 % slam freq 20hz(0.05s)
        idx_bad4slam = [idx_bad4slam; idx_tmp];
        time_diff_bad4slam = [time_diff_bad4slam; t_diff];
        break;
    end
    idx_gtruth4slam = [idx_gtruth4slam; idx_tmp];
    time_diff_gtruth4slam = [time_diff_gtruth4slam; t_diff];
end
if (size(idx_gtruth4slam,1) ~= size(data_slam_taligned,1))
    disp 'something wrong with time alignment between ground truth and data slam.'    
    disp 'check your program.'
    return
end

gtruth_taligned2slam = data_gtruth(idx_gtruth4slam, 1:8);
%% adjust timestamp, so it starts from 0 sec
% data_slam_taligned(:,1) = (data_slam_taligned(:,1) - data_slam_taligned(1,1)) * 1.0e-09;
% gtruth_taligned2slam(:,1) = (gtruth_taligned2slam(:,1) - gtruth_taligned2slam(1,1)) * 1.0e-09;
%%
data_slam_taligned(:,1) = data_slam_taligned(:,1) * 1.0e-09;
gtruth_taligned2slam(:,1) = gtruth_taligned2slam(:,1) * 1.0e-09;
% data_gtruth4slam(1,1) - data_gtruth4slam(end,1)
end