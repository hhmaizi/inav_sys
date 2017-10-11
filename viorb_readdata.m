%% data read
path = '\\10.10.194.34\foaa\Teddy_Zhang\tmp\inav_analysis\test0929\'; % #timestamp p_RS_R_x  p_RS_R_y p_RS_R_z q_RS_w q_RS_x q_RS_y q_RS_z v_RS_R_x v_RS_R_y v_RS_R_z
filenames_slam = {'M01_FT_VI.txt', 'M03_FT_VI.txt', 'V103_FT_VI.txt', 'V203_FT_VI.txt'};
filenames_imu = {'M01_FT_IMU01_whole.txt', 'M01_FT_IMU01_05.txt', 'M01_FT_IMU02_05.txt', 'M01_FT_IMU03_05.txt', 'M01_FT_IMU04_05.txt';...
    '','','','','';...
   'V103_FT_VI.txt', 'V103_FT_IMU01_05.txt', 'V103_FT_IMU02_05.txt','V103_FT_IMU03_05.txt','V103_FT_IMU04_05.txt';...
   'V203_FT_VI.txt', 'V203_FT_IMU01_05.txt', 'V203_FT_IMU02_05.txt','V203_FT_IMU03_05.txt', 'V203_FT_IMU04_05.txt'};

filenames_imucut = {'M01_INAV_IMU01_whole.txt', 'M01_INAV_IMU01_05.txt', 'M01_INAV_IMU02_05.txt', 'M01_INAV_IMU03_05.txt', 'M01_INAV_IMU04_05.txt';...
    '','','','','';...
   'V103_INAV_VI.txt', 'V103_INAV_IMU01_05.txt', 'V103_INAV_IMU02_05.txt','V103_INAV_IMU03_05.txt','V103_INAV_IMU04_05.txt';...
   'V203_INAV_VI.txt', 'V203_INAV_IMU01_05.txt', 'V203_INAV_IMU02_05.txt','V203_INAV_IMU03_05.txt', 'V203_INAV_IMU04_05.txt'};
% m01, m03, v103, v203
datasetidx = 1; dataset_subidx = 2;
data_imucut = load([path, filenames_imucut{datasetidx, dataset_subidx}]);
data_imucut(:,1) = data_imucut(:,1)*10^(-9);
imu_start = data_imucut(1,1) ; imu_end = data_imucut(end,1);
data_vi=load([path filenames_slam{datasetidx}]); % time p_x p_y p_z q_x q_y q_z q_w
% data_filter=load([path 'InavTrajectorym0303.txt']);
data_vi(:,1)=data_vi(:,1)*10^(-9);
data_imu=load([path filenames_imu{datasetidx, dataset_subidx}]); % time  p_x p_y p_z q_  w q_x q_y q_z
data_imu(:,1)=data_imu(:,1)*10^(-9);
slam=data_imu; % read origin_slam data
quat_slam = slam(:, 5:8);
data_slam=data_vi; %read imu_slam data
quat_slam = data_slam(:, 5:8);