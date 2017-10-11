
%%
path = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\test0929\INAV\';
f01 = [path, 'data01.txt'];
f02 = [path, 'data02.txt'];
data01 = load(f01);
data02 = load(f02);

%%
% align time stamp
if (data01(1,1) >= data02(1,1))
    idxstart_data02 = find(data02(:,1) >= data01(1,1),1);
    idxstart_data01 = 1;
else
    idxstart_data01 = find(data01(:,1) >= data02(1,1),1);
    idxstart_data02 = 1;
end
%%
figure
plot3(data01(idxstart_data01:end, 2), data01(idxstart_data01:end,3), data01(idxstart_data01:end,4), 'r')
hold on;
plot3(data02(idxstart_data02:end,2), data02(idxstart_data02:end,3), data02(idxstart_data02:end,4))
title('3d compare')

%%
figure()
subplot(3,1,1)
plot(data01(:,2)-data02(:,2),'r')
subplot(3,1,2)
plot(data01(:,3)-data02(:,3),'g')
subplot(3,1,3)
plot(data01(:,3)-data02(:,3),'b')

