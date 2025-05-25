addpath(genpath("TwistCalculation/twistAndScrew"));
addpath(genpath("TwistCalculation/drawPic"));
addpath(genpath("Coordinate"));
addpath(genpath("Matrix"));
addpath(genpath("Utils"));
addpath(genpath("simple"));
addpath(genpath("SphereTools"));

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

% foot 三个分支变换矩阵（共用）
T_foot_front = [1 0 0 0.180; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot_mid   = [1 0 0 0.000; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot_hind  = [1 0 0 -0.090; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot = {T_foot_front, T_foot_mid, T_foot_hind};

% 左腿变换矩阵列表
T_list_left = {
    [ 1 0 0 0; 0 1 0 0.064452; 0 0 1 -0.102700; 0 0 0 1],
    [ 0.984744 0 -0.174010 0; 0 1 0 0.052000; 0.174010 0 0.984744 -0.030465; 0 0 0 1],
    [ 1 0 0 0.025001; 0 1 0 0; 0 0 1 -0.124120; 0 0 0 1],
    [ 0.984744 0 0.174010 -0.078273; 0 1 0 0.002149; -0.174010 0 0.984744 -0.177340; 0 0 0 1],
    [ 1 0 0 0; 0 1 0 -0.000094; 0 0 1 -0.300010; 0 0 0 1],
    [ 1 0 0 0; 0 1 0 0; 0 0 1 -0.017558; 0 0 0 1],
};

% 右腿变换矩阵列表
T_list_right = {
    [ 1 0 0 0; 0 1 0 -0.064452; 0 0 1 -0.102700; 0 0 0 1],
    [ 0.984744 0 -0.174010 0; 0 1 0 -0.052000; 0.174010 0 0.984744 -0.030465; 0 0 0 1],
    [ 1 0 0 0.025001; 0 1 0 0; 0 0 1 -0.124120; 0 0 0 1],
    [ 0.984744 0 0.174010 -0.078273; 0 1 0 -0.002149; -0.174010 0 0.984744 -0.177340; 0 0 0 1],
    [ 1 0 0 0; 0 1 0 0.000094; 0 0 1 -0.300010; 0 0 0 1],
    [ 1 0 0 0; 0 1 0 0; 0 0 1 -0.017558; 0 0 0 1],
};

% 世界坐标系初始化
C_world = showWorldCoordinate(0.1);

% 左腿主链
[lineList_left, T_left_foot] = drawLeg(T_list_left, eye(4), C_world, 'L', 'b');

% 左脚三趾
for i = 1:length(T_foot)
    [~, ~] = drawLeg({T_foot{i}}, T_left_foot, C_world, sprintf('L_f%d', i), 'b');
end

% 右腿主链
[lineList_right, T_right_foot] = drawLeg(T_list_right, eye(4), C_world, 'R', 'r');

% 右脚三趾
for i = 1:length(T_foot)
    [~, ~] = drawLeg({T_foot{i}}, T_right_foot, C_world, sprintf('R_f%d', i), 'm');
end

% 美化显示
view(3);
axis equal;
title('Left and Right Leg Kinematic Chain Visualization');
