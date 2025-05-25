function [] = showLine(AxisList, line, color)
% showLine - 在三维空间中绘制点序列首尾相连的线段
%
% 输入参数：
%   AxisList - 点结构体数组（每个点有字段 X, Y, Z）
%              连线将自动连接 AxisList(i) → AxisList(i+1)
%   line     - 绘图样式，如 '-', '--', '-o'
%   color    - 绘图颜色，如 'r', 'b'，或 RGB 三元组

% demo
% 构造坐標系首尾相連线段
% lineList = [C_world.p0, C_body1.p0, C_body2.p0, C_body3.p0];
% showLine(lineList, '-', 'b');

n = length(AxisList);
if n < 2
    warning('AxisList 至少需要两个点才能连线');
    return;
end

for i = 1:n-1
    p0 = AxisList(i);
    p1 = AxisList(i+1);
    x = [p0.X, p1.X];
    y = [p0.Y, p1.Y];
    z = [p0.Z, p1.Z];
    plot3(x, y, z, line, 'Color', color);
    hold on;
end
end
