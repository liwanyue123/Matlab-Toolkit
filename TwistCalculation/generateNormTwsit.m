function [S1] = generateNormTwsit(axis, RorP)
% generateNormTwsit - 生成标准单位螺旋向量
% 输入：
%   axis     - 字符串 'X'、'Y'、'Z'，表示旋量的轴方向
%   RorP - 字符串 
% 输出：
%   S1       - 1x6 的行向量，前3位为 S_Rotational  ，后3位为 S_Prismatic

% demo
% twistV = generateNormTwsit('X', 'Rotational');  % 得到速度旋量 [1 0 0  0 0 0]
% twistF = generateNormTwsit('Y', 'Rotational');  % 得到力旋量   [0 1 0  0 0 0]


% 初始化为零向量
S_Prismatic = [0, 0, 0];
S_Rotational = [0, 0, 0];

% 判断旋量类型
if strcmp(RorP, 'Rotational')  % 力旋量（对偶变换）
    switch upper(axis)
        case 'X'
            S_Rotational = [1, 0, 0];
        case 'Y'
            S_Rotational = [0, 1, 0];
        case 'Z'
            S_Rotational = [0, 0, 1];
        otherwise
            error('非法轴方向，应为 X, Y, 或 Z');
    end
elseif strcmp(RorP, 'Prismatic')  % 速度旋量
    switch upper(axis)
        case 'X'
            S_Prismatic = [1, 0, 0];
        case 'Y'
            S_Prismatic = [0, 1, 0];
        case 'Z'
            S_Prismatic = [0, 0, 1];
        otherwise
            error('非法轴方向，应为 X, Y, 或 Z');
    end
else
    error('非法旋量类型，应为 "Prismatic"（平动）或 "Rotational"（转动）');
end

S1 = [S_Rotational,S_Prismatic]';
end
