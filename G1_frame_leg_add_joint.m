addpath(genpath("TwistCalculation/twistAndScrew"));
addpath(genpath("TwistCalculation/drawPic"));
addpath(genpath("Coordinate"));
addpath(genpath("Matrix"));
addpath(genpath("Utils"));
addpath(genpath("simple"));
addpath(genpath("SphereTools"));

% 单纯的绘制矩阵T
function [lineList, T_cum] = drawLeg(T_list, T_base, C_world, labelPrefix, color)
    T_cum = T_base;
    lineList = [];
    lineList = [lineList, getCoordDotFromT(T_base)];  % 取出初始坐标
    for i = 1:length(T_list)
        T_cum = T_cum * T_list{i};
        Ci = genCoordinateCoord(C_world, T_cum);
        showCoordinate(Ci, sprintf('%s_{%d}', labelPrefix, i), 0.1);
        lineList = [lineList, Ci.p0(:)];
    end
    showLine(lineList, '-', color,3);
end

function [lineList, T_cum] = drawLegJoint(leg_struct, T_base, C_world, labelPrefix, color)
    T_cum = T_base;
    lineList = [];
    lineList = [lineList, getCoordDotFromT(T_base)];  % 初始坐标点
   
    n = length(leg_struct);
       % drawLegJoint函数中
    for i = 1:n
        T_joint = calTranMatrixFromR(leg_struct(i).type, leg_struct(i).theta);
        T_cum = T_cum * leg_struct(i).T * T_joint;  % 矩阵乘法顺序修改
        Ci = genCoordinateCoord(C_world, T_cum);
        showCoordinate(Ci, sprintf(leg_struct(i).name, labelPrefix, i), 0.1);
        lineList = [lineList, Ci.p0(:)];
    end

    
    % 绘制连接线
    showLine(lineList, '-', color, 2);
end
 

% foot 三个分支变换矩阵（共用）
T_foot_front = [1 0 0 0.180; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot_mid   = [1 0 0 0.000; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot_hind  = [1 0 0 -0.090; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot = {T_foot_front, T_foot_mid, T_foot_hind};


 
% 左腿
leg_left = struct(...
    'type', {'Y','X','Z','Y','Y','X'}, ...
    'theta', {0, 0, 0, 0, 0, 0}, ...
    'name', {'hip_{pitch}','hip_{roll}','hip_{yaw}','knee','ankle_{pitch}','ankle_{roll}'}, ...
    'T', {...
        [ 1 0 0 0; 0 1 0 0.064452; 0 0 1 -0.102700; 0 0 0 1],...
        [ 0.984744 0 -0.174010 0; 0 1 0 0.052000; 0.174010 0 0.984744 -0.030465; 0 0 0 1],...
        [ 1 0 0 0.025001; 0 1 0 0; 0 0 1 -0.124120; 0 0 0 1],...
        [ 0.984744 0 0.174010 -0.078273; 0 1 0 0.002149; -0.174010 0 0.984744 -0.177340; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 -0.000094; 0 0 1 -0.300010; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 0; 0 0 1 -0.017558; 0 0 0 1],...
    } ...
);



% 右腿
leg_right = struct(...
    'type', {'Y','X','Z','Y','Y','X'}, ...
    'theta', {0, 0, 1.57, 0, 0, 0}, ...
    'name', {'hip_{pitch}','hip_{roll}','hip_{yaw}','knee','ankle_{pitch}','ankle_{roll}'}, ...
    'T', {...
        [ 1 0 0 0; 0 1 0 -0.064452; 0 0 1 -0.102700; 0 0 0 1],...
        [ 0.984744 0 -0.174010 0; 0 1 0 -0.052000; 0.174010 0 0.984744 -0.030465; 0 0 0 1],...
        [ 1 0 0 0.025001; 0 1 0 0; 0 0 1 -0.124120; 0 0 0 1],...
        [ 0.984744 0 0.174010 -0.078273; 0 1 0 -0.002149; -0.174010 0 0.984744 -0.177340; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 0.000094; 0 0 1 -0.300010; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 0; 0 0 1 -0.017558; 0 0 0 1],...
    } ...
);


% 世界坐标系初始化
C_world = showWorldCoordinate(0.1);

% 左腿主链
[lineList_left, T_left_foot]  = drawLegJoint(leg_left, eye(4), C_world, 'L', 'b');

% 左脚三趾
for i = 1:length(T_foot)
    [~, ~] = drawLeg({T_foot{i}}, T_left_foot, C_world, sprintf('L_{f%d}', i), 'b');
end

% 右腿主链
[lineList_right, T_right_foot]= drawLegJoint(leg_right, eye(4), C_world, 'R', 'r');

% 右脚三趾
for i = 1:length(T_foot)
    [~, ~] = drawLeg({T_foot{i}}, T_right_foot, C_world, sprintf('R_{f%d}', i), 'r');
end

% 美化显示
% figure('Position', [100, 100, 800, 600]);  % 左下角(x,y), 宽, 高
view(37.5, 30);   % 默认 3D 视角
axis equal;
title('Left and Right Leg Kinematic Chain Visualization');
