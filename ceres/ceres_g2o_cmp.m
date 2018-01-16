%% comparing ceres and g2o for current frame
x = nav(:,2); y = nav(:,3); z = nav(:,4);
xini = x(1:3:end); xceres = x(2:3:end); xg2o = x(3:3:end);
yini = y(1:3:end); yceres = y(2:3:end); yg2o = y(3:3:end);
zini = z(1:3:end); zceres = z(2:3:end); zg2o = z(3:3:end);
% x axis
figure; plot(xini,'r');
hold on; plot(xceres, 'g');
plot(xg2o, 'b');
title('x axis')
legend('ini', 'ceres', 'g2o')
hold off
% y axis
figure; plot(yini,'r');
hold on; plot(yceres, 'g');
plot(yg2o, 'b');
title('y axis')
legend('ini', 'ceres', 'g2o')
hold off
% z axis
figure; plot(zini,'r');
hold on; plot(zceres, 'g');
plot(zg2o, 'b');
title('z axis')
legend('ini', 'ceres', 'g2o')
hold off

%% 