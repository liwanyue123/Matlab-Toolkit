addpath(genpath("TwistCalculation"));%旋量相关
addpath(genpath("Coordinate"));%计算显示坐标系相关
addpath(genpath("Matrix"));%矩阵相关
addpath(genpath("Utils"));%矩阵相关
addpath(genpath("simple"));%符号处理相关

addpath(genpath("SphereTools"));%

clear

 
 
q=[0.86023,0.415538,0.2661,-0.128541]
 


R = quat2rotm(q)

% 1.rotation matrix

% R=[-0.37426245  0.92696559  0.02573561;
%   0.92144239  0.37486649 -0.10207357;
%  -0.1042661  -0.01448843 -0.99444377]

% --------------------------------------
% Substitute specific values for analysis
 
%  绘制世界坐标系
C_world=showWorldCoordinate(2);

C_body1=genCoordinateCoord(C_world,expandMatTo4Dim(R));
showCoodinate(C_body1,'C_{body}',1)
% 设置相机的角度
azimuth = 45; % 方位角
elevation = 30; % 仰角
view(azimuth, elevation); % 设置相机角度

axis equal


