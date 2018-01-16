vst_tstamp = [1403715930.0; 1403715950.0; 1403715970.0; 1403715925.0];
% 001 xaxis: -2.5, 2; yaxis -2, 4; zaxis:  0.8, 2.6
% 002 xaxis: -2.5, 2; yaxis -2, 4; zaxis:  0.8, 2.6
% 003 xaxis: -2.5, 2; yaxis -2, 4; zaxis:  0.8, 2.6
%%
% ymin = -2.5; ymax = 2;
% ymin = -2; ymax = 4;
ymin = 0.8; ymax = 2.6;
data_id = 3; duration = 5;
hold on;
plot(vst_tstamp(data_id)*ones(100), linspace(ymin, ymax, 100), 'm');
plot((vst_tstamp(data_id) + duration)*ones(100), linspace(ymin, ymax, 100), 'm');
hold off