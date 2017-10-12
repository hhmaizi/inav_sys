%clc
%clear all
close all
% %% TEST
% image_gap=[];
% for i=2:1:length(data(:,1))
%     image_gap(end+1,1)=(data(i,1)-data(i-1,1))/1000/1000/1000;
%     if(abs(image_gap(end,1)-0.05)>0.01)
%         i
%         image_gap(end,1)
%     end
% end

%% data read
path = '\\10.10.194.34\foaa\Oliver_Sun\tmp\tmpMH01\'; % #timestamp p_RS_R_x  p_RS_R_y p_RS_R_z q_RS_w q_RS_x q_RS_y q_RS_z v_RS_R_x v_RS_R_y v_RS_R_z

data_filter=load([path 'FrameTrajectory0.txt']); % time p_x p_y p_z q_x q_y q_z q_w
data_filter(:,1)=data_filter(:,1)/1000/1000/1000;
data_slam=load([path 'FrameTrajectory1.txt']); % time  p_x p_y p_z q_  w q_x q_y q_z
data_slam(:,1)=data_slam(:,1)/1000/1000/1000;

% data_optimize=load([path 'PoseOptimization.txt']);
data_optimize=load(['\\10.10.194.34\foaa\Oliver_Sun\tmp\' 'PoseOptimization.txt']);
slam=data_slam; % read origin_slam data
filter=data_filter; %read imu_slam data
origin=data; %read ground_truth Tiw

line2show=3; %x,y,z

pci = [-0.0216,-0.0647,0.0098];
rci=[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0];
qci =  quatconj(dcm2quat(rci(1:3,1:3))); %左乘矩阵变换，故而默认右乘情况下要加个转置
qiw=[0.5341,-0.1530,-0.8274,-0.0822];
piw=[4.6883,-1.7869,0.7833];
imu_cam=-inv(rci(1:3,1:3))*pci';
%% slam2world
slam_worldimu=[];
slam_quat=[];
%begin frame
for i=1:1:length(origin(:,1))
   if(((origin(i,1)/1000/1000/1000-slam(1,1))) > 0 && (origin(i+1,1)/1000/1000/1000-slam(1,1)) > 0)
        fprintf('timestamp error!!\r\n');
   end
   if(abs((origin(i,1)/1000/1000/1000-slam(1,1)))<abs((origin(i+1,1)/1000/1000/1000-slam(1,1))))
       piw=origin(i,2:4);
       qiw=origin(i,5:8);
       i
       break;
   end
end
%slam cam_vision -> imu_world
% piw: imu in world, qiw: imu attitude in world, pci: cam in body frame.
% qci: body attitude in cam
% imu_cam: body in cam frame.
for i=1:1:length(slam(:,1))
    % body in slam
    imu_vision=quatrotate(quatconj(slam(i,5:8)),imu_cam')+slam(i,2:4); %frame imu pos in vision coordinate 
    % body in slam(from slam origin to body) expressed by body frame.
    slam_worldimu(end+1,:) = quatrotate(quatconj(qci),imu_vision);
    slam_worldimu(end,:)=slam_worldimu(end,:)+pci;
    slam_worldimu(end,:)=quatrotate(quatconj(qiw), slam_worldimu(end,:))+piw;
end

for i=1:1:length(slam(:,1))
    slam_quat(end+1,:)=quatmultiply(qiw,quatmultiply(qci,quatmultiply(slam(i,5:8),quatconj(qci))));
end
%tmp version
% for i=1:1:length(slam(:,1))
%     imu_vision=quatrotate(quatconj(slam(i,[8,5,6,7])),imu_cam')+slam(i,2:4); %frame imu pos in vision coordinate 
%     slam_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
%     slam_worldimu(end,:)=slam_worldimu(end,:)+pci;
%     slam_worldimu(end,:)=quatrotate(quatconj(qiw), slam_worldimu(end,:))+piw;
% end

%% filter2world
filter_worldimu=[];
filter_quat=[];
origin_frame_begin=1;
if 1
%begin frame
for i=1:1:length(origin(:,1))
    if(((origin(i,1)/1000/1000/1000-filter(1,1))) > 0 && (origin(i+1,1)/1000/1000/1000-filter(1,1)) > 0)
        fprintf('timestamp error!!\r\n');
    end
   if(abs((origin(i,1)/1000/1000/1000-filter(1,1)))<abs((origin(i+1,1)/1000/1000/1000-filter(1,1))))
       piw=origin(i,2:4);
       qiw=origin(i,5:8);
       i
       origin_frame_begin=i;
       break;
   end
end
%filter cam_vision -> imu_world
% for i=1:1:length(filter(:,1))
%     imu_vision=quatrotate(quatconj(filter(i,[8,5,6,7])),imu_cam')+filter(i,2:4);
%     filter_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
%     filter_worldimu(end,:)=filter_worldimu(end,:)+pci;
%     filter_worldimu(end,:)=quatrotate(quatconj(qiw), filter_worldimu(end,:))+piw;
% end
% for i=1:1:length(filter(:,1))
%     filter_quat(end+1,:)=quatmultiply(qiw,quatmultiply(qci,quatmultiply(filter(i,[8,5,6,7]),quatconj(qci))));pi
% end
%tmp version
for i=1:1:length(filter(:,1))
    imu_vision=quatrotate(quatconj(filter(i,5:8)),imu_cam')+filter(i,2:4);
    filter_worldimu(end+1,:)=quatrotate(quatconj(qci),imu_vision);
    filter_worldimu(end,:)=filter_worldimu(end,:)+pci;
    filter_worldimu(end,:)=quatrotate(quatconj(qiw), filter_worldimu(end,:))+piw;
end
for i=1:1:length(filter(:,1))
    filter_quat(end+1,:)=quatmultiply(qiw,quatmultiply(qci,quatmultiply(filter(i,5:8),quatconj(qci))));
end

else
    for i=1:1:length(filter(:,1))
        filter_worldimu(end+1,:)=filter(i,2:4);
    end
end
%% EKF2world
%imu imu_world
% for i=1:1:length(predict(:,1))
%     predict(i,2:4)=quatrotate(quatconj(qiw),predict(i,2:4))+piw;
% end
%origin imu_world -> cam_world
% for i=1:1:length(origin(:,1))
%     origin(:,2:4)=quatrotate(quatconj(origin(:,5:8)),pci)+origin(:,2:4);
% end

%% variance calculation
filter_pointer=1;
filter_nocmp=0;
filter_variance=0;
filter_angle_diff=0;
slam_pointer=1;
slam_nocmp=0;
slam_variance=0;
slam_angle_diff=0;
count=0;
a=0;
for i=1:1:length(origin(:,1))-1
     t=double(origin(i,1)/1000/1000/1000);
     t_plus1=double(origin(i+1,1)/1000/1000/1000);
    if(filter_pointer<=length(filter(:,1)))
        while(filter_pointer<=length(filter(:,1)) && (t-filter(filter_pointer,1)) > 0 && (t_plus1-filter(filter_pointer,1)) > 0)
            filter_pointer=filter_pointer+1;
            filter_nocmp=filter_nocmp+1;
        end
        if(filter_pointer<=length(filter(:,1)) && abs(t-filter(filter_pointer,1))<abs(t_plus1-filter(filter_pointer,1)))
            filter_variance=filter_variance+(norm(filter_worldimu(filter_pointer,1:3)-origin(i,2:4)))^2;
             angle_filter=acosd(abs(filter_quat(filter_pointer,1)));
             angle_origin=acosd(abs(origin(i,5)));
             filter_angle_diff=filter_angle_diff+abs(angle_filter-angle_origin);
            filter_pointer=filter_pointer+1;
        end
    end
    if(slam_pointer<=length(slam(:,1)))
        while((t-slam(slam_pointer,1)) > 0 && (t_plus1-slam(slam_pointer,1)) > 0)
            slam_pointer=slam_pointer+1;
            slam_nocmp=slam_nocmp+1;
        end
        if(abs(t-slam(slam_pointer,1))<abs(t_plus1-slam(slam_pointer,1)))
            slam_variance=slam_variance+(norm(slam_worldimu(slam_pointer,1:3)-origin(i,2:4)))^2;
             angle_slam=acosd(abs(slam_quat(slam_pointer,1)));
             angle_origin=acosd(abs(origin(i,5)));
             slam_angle_diff=slam_angle_diff+abs(angle_slam-angle_origin);
            slam_pointer=slam_pointer+1;
        end
    end
end
filter_variance=sqrt(filter_variance/(filter_pointer-2-filter_nocmp))
slam_variance=sqrt(slam_variance/(slam_pointer-2-slam_nocmp))
filter_angle_diff=filter_angle_diff/(filter_pointer-1-filter_nocmp)
slam_angle_diff=slam_angle_diff/(slam_pointer-1-slam_nocmp)

sum=0;
for i=2:1:length(filter_worldimu(:,1))
    sum=sum+norm(filter_worldimu(i,1:3)-filter_worldimu(i-1,1:3));
end
loop_dis=norm(filter_worldimu(end,1:3)-filter_worldimu(1,1:3));
percent=loop_dis/sum

% op_length=length(data_optimize(:,1));
% op_diff=[];
% for i=1:1:op_length
%     if(data_optimize(i,1)~=data_optimize(i-1,1))
%        op_diff(end+1,1)=data_optimize(i-1,1);
%        a_dis=norm(data_optimize(i-1,3:5));
%        b_dis=norm(data_optimize(i-2,3:5));
%        op_diff(end,2)=abs(a_dis-b_dis);
%     end
% end
data_record=[];
for i=1:1:length(data_optimize(:,1))
    imu_vision=data_optimize(i,3:5);%quatrotate(quatconj(data_optimize(i,6:9)),imu_cam')+data_optimize(i,3:5);
    data_record(end+1,:)=quatrotate(quatconj(qci),imu_vision);
    data_record(end,:)=data_record(end,:)+pci;
    data_record(end,:)=quatrotate(quatconj(qiw), data_record(end,:))+piw;
    data_optimize(i,3:5)=data_record(i,:);
    data_optimize(i,10) = data_optimize(i,10)/1000/1000/1000;
end
op_length=length(data_optimize(:,1));
opt_1=[];
opt_2=[];
opt_3=[];
for i=4:1:op_length
    if(data_optimize(i,1)~=data_optimize(i-1,1))
       if(data_optimize(i-1,2)==2)
           opt_1(end+1,1)=data_optimize(i-1,10);
           opt_1(end,2)=data_optimize(i-1,line2show+2);
           if(data_optimize(i-1,1)==data_optimize(i-2,1))
               opt_2(end+1,1)=data_optimize(i-2,10);
               opt_2(end,2)=data_optimize(i-2,line2show+2);
               opt_3(end+1,1)=data_optimize(i-3,10);
               opt_3(end,2)=data_optimize(i-3,line2show+2);
           end       
       else
           opt_2(end+1,1)=data_optimize(i-1,10);
           opt_2(end,2)=data_optimize(i-1,line2show+2);
           opt_3(end+1,1)=data_optimize(i-2,10);
           opt_3(end,2)=data_optimize(i-2,line2show+2);
       end
    end
end
%% figure draw
figure(1)
plot(1:length(filter_worldimu(:,1)),filter_worldimu(1:end,line2show),'b');
%plotyy(1:length(filter_worldimu(:,1)),filter_worldimu(1:end,line2show),op_diff(1:end,1),op_diff(1:end,2));
%plot(origin_frame_begin:length(origin(:,1)),origin(origin_frame_begin:end,line2show),'k');

% plot(opt_1(:,1),opt_1(:,2),'g');
% hold on
% plot(opt_2(:,1),opt_2(:,2),'b');
% hold on
% plot(opt_3(:,1),opt_3(:,2),'r');
% hold on
% plot(double(origin(1:end,1))/1000/1000/1000,origin(1:end,line2show + 1),'k');

figure(2)
% for i=1:length(filter(1:end,1))
%     if(filter(i,9)==1)
%         scatter(filter(i,1),filter_worldimu(i,line2show),'ro');
%         hold on
%     else
%         scatter(filter(i,1),filter_worldimu(i,line2show),'go');
%         hold on
%     end
% end
plot(filter(1:end,1),filter_worldimu(1:end,line2show),'b');
hold on
plot(slam(:,1),slam_worldimu(:,line2show),'g');
hold on
plot(double(origin(1:end,1))/1000/1000/1000,origin(1:end,line2show + 1),'r');
legend('filter','slam','groundtruth');


%%plot3d
% scatter3(slam_worldimu(:,1),slam_worldimu(:,2),slam_worldimu(:,3),'b');
% hold on
% scatter3(origin(:,2),origin(:,3),origin(:,4),'r');

