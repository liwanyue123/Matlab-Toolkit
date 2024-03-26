addpath(genpath("../Utils"));%矩阵相关
addpath(genpath("drawPic"));%矩阵相关
addpath(genpath("twistAndScrew"));%矩阵相关

norm_max_tau=20;



F_A = [0; 0; 0; 4; 2; 3];% force旋量[w v]
p_A = [0; 2; 0]; % force位置

% target positon
p_B=[0;0;0];
r_BA=p_A-p_B%force arm
hat(r_BA)*[4;0;0]
showType = 'all';
rate = 0.5;

F_B=calScrewFromOtherScrew(F_A,r_BA)
hold on
drawVector(p_B, r_BA)
showScrew(p_A,F_A ,showType,rate,'blue',norm_max_tau)
showScrew(p_B,F_B ,showType,rate,'red',norm_max_tau)

% 设置相机的角度
azimuth = 45; % 方位角
elevation = 30; % 仰角
view(azimuth, elevation); % 设置相机角度

axis equal