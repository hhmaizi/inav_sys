function [slam_tmp, inav_tmp] = align_timestamp(data_slam, data_inav)

tstamp0 = max(data_slam(1,1), data_inav(1,1));
tstamp1 = min(data_slam(end, 1), data_inav(end, 1));

slam_tmp = [];
inav_tmp = [];
for idx = 1: size(data_slam, 1)
    if data_slam(idx, 1) < tstamp0
        break
    end
    if data_slam(idx, 1) > tstamp1
        break
    end
    [tmp_diff, idx_inav] = min(abs(data_slam(idx,1) - data_inav(:,1)));
    slam_tmp(end+1, :) = data_slam(idx, :);
    inav_tmp(end+1, :) = data_inav(idx_inav, :);
end
end