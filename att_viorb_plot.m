%% plotting
figure
subplot(3,1,1)
plot(data_gtruth4slam(:,1), anz_gd(:), 'r')
hold on
plot(data_gtruth4slam(:,1), any_gd(:), 'g')
plot(data_gtruth4slam(:,1), anx_gd(:), 'b')
title('ground truth Ref')
subplot(3,1,2)
plot(data_slam(:,1), anz_rb_slam, 'r')
hold on
plot(data_slam(:,1), any_rb_slam, 'g')
plot(data_slam(:,1), anx_rb_slam, 'b')
title('slam aligned to Ref')

subplot(3,1,3)
plot(data_slam(:,1), anz_slam, 'r')
hold on
plot(data_slam(:,1), any_slam, 'g')
plot(data_slam(:,1), anx_slam, 'b')
title('slam unaligned')
hold off

%% abs plotting

figure
subplot(3,1,1)
plot(data_gtruth4slam(:,1), abs(anz_gd(:)), 'r')
hold on
plot(data_gtruth4slam(:,1), abs(any_gd(:)), 'g')
plot(data_gtruth4slam(:,1), abs(anx_gd(:)), 'b')
title('ground truth Ref')
subplot(3,1,2)
plot(data_slam(:,1), abs(anz_rb_slam), 'r')
hold on
plot(data_slam(:,1), abs(any_rb_slam), 'g')
plot(data_slam(:,1), abs(anx_rb_slam), 'b')
title('slam aligned to Ref ')

subplot(3,1,3)
plot(data_slam(:,1), abs(anz_slam), 'r')
hold on
plot(data_slam(:,1), abs(any_slam), 'g')
plot(data_slam(:,1), abs(anx_slam), 'b')
title('slam unaligned')
hold off