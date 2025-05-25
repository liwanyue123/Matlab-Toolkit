function [S1] = generateTwsit(r1, s1, h, isShow)
% generateTwsit - 生成螺旋线速度向量（twist）
%
% 输入参数:
%   r1     - 一个点的位置向量，表示螺旋轴上的一点（3x1 向量）
%   s1     - 单位方向向量，表示螺旋轴方向（3x1 向量）
%   h      - 螺旋参数（pitch），控制旋转与平移的关系
%   isShow - 布尔值，是否绘制向量图（true 则调用 showVector 可视化）
%
% 输出参数:
%   S1 - 螺旋线速度向量（twist），是一个 6x1 列向量，形式为 [ω; v]

% 如果开启了可视化，则显示向量
if isShow == true
    % 绘制从原点到 r1 的向量（通常用于表示位置）
    showVector([0, 0, 0], r1, 'g');
    
    % 从 r1 开始，绘制方向向量 s1（用于表示轴的方向）
    showVector(r1, s1, 'r');
end

% 计算旋转轴与点位置的叉乘，得到瞬时速度的线速度部分
s10 = cross(r1, s1);

% 构造 twist 向量：[s1; s10 + h * s1]，为 6x1 格式
% s1 为旋转部分（角速度 ω），s10 + h*s1 为平移部分（线速度 v）
S1 = [s1, s10 + h * s1]';
end
