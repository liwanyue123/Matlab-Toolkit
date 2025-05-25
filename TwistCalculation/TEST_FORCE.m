addpath(genpath("Coordinate"));%计算显示坐标系相关
addpath(genpath("Utils"));%矩阵相关
addpath(genpath("TwistCalculation/twistAndScrew"));%旋量相关
addpath(genpath("TwistCalculation/drawPic"));%旋量相关

norm_max_tau=10;
showType = 'all';
rate = 0.5;
showWorldCoordinate(1)

% F_A = [0; 0; 0; 0; 3; 4];% force旋量[w v]
% p_A = [0; 2; 0]; % force位置
% 
% % target positon
% p_B=[0;0;0];
% r_BA=p_A-p_B%force arm
% hat(r_BA)*[4;0;0]
% 
% 
% F_B=calScrewFromOtherScrew(F_A,r_BA)
% hold on
% drawLineArrow(p_B, r_BA,'grey')
% showScrew(p_A,F_A ,showType,rate,'blue',norm_max_tau)
% showScrew(p_B,F_B ,showType,rate,'red',norm_max_tau)





p_B=[0;0;0]; 
tau_B=[8;0;0];
showScrew(p_B,[tau_B;0;0;0] ,showType,rate,'red',norm_max_tau,'B')


p_A = [0; 2; 0];
r_AB=p_B-p_A;
drawLineArrow(p_A, r_AB,'grey',1)
f_A=hat(r_AB)*tau_B

showScrew(p_A,[0;0;0;f_A] ,showType,rate,'blue',norm_max_tau,'A')


% 设置相机的角度
azimuth = 45; % 方位角
elevation = 30; % 仰角
view(azimuth, elevation); % 设置相机角度

axis equal