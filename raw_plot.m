%% 
% pos_gd, pos_slam
figure;
subplot(3,1,1)
plot(pos_gd(:,1), 'r')
hold on
plot(pos_slam(:,1), 'b')
legend('ground truth', 'slam')
%
subplot(3,1,2)
plot(pos_gd(:,2), 'r')
hold on
plot(pos_slam(:,2), 'b')
%
subplot(3,1,3)
plot(pos_gd(:,3), 'r')
hold on
plot(pos_slam(:,3), 'b')
hold off