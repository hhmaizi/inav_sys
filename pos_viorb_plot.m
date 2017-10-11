figure
subplot(3,1,1)
plot(data_gtruth4slam(:,1), pos_gd(:,1), 'r')
hold on
plot(data_slam(:,1), pos_rb_slam(:,1), 'g')
title('xaxis ground truth vs slam')
legend('x ground truth', 'x slam')

subplot(3,1,2)
plot(data_gtruth4slam(:,1), pos_gd(:,2), 'r')
hold on
plot(data_slam(:,1), pos_rb_slam(:,2), 'g')
title('yaxis ground truth vs slam')
legend('y ground truth', 'y slam')

subplot(3,1,3)
plot(data_gtruth4slam(:,1), pos_gd(:,3), 'r')
hold on
plot(data_slam(:,1), pos_rb_slam(:,3), 'g')
title('zaxis ground truth vs slam')
legend('z ground truth', 'z slam')