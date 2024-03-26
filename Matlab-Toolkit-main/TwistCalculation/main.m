addpath(genpath("../Utils"));%矩阵相关
% 初始位置
p0 = [0; 0; 0];

% 初始速度旋量
V0 = [0.5; 0.3; 0.7; 0.2; 0.4; 0.6];

% 场范围
distance = 1;
num=2;
range =num*distance;
% 要显示的模式：'all', 'w', 'v', 'null'
showType = 'v';
% showType = 'all';
% 缩放比例
rate = 0.5;

hold on
[x, y] = meshgrid(-range:distance:range, -range:distance:range);
z = p0(3) * ones(size(x));
surf(x, y, z, 'FaceAlpha', 0.5);

% 调用 showTwistFiled 函数显示速度场
showTwistPlane(p0, V0, num, distance, showType, rate);
% showTwistFiled(p0, V0, num, distance, showType, rate);

% 设置相机的角度
azimuth = 45; % 方位角
elevation = 30; % 仰角
view(azimuth, elevation); % 设置相机角度

axis equal
