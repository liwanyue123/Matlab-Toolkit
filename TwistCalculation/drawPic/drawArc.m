function [] = drawCircle(C,R,normal)
% draw圆
% 圆心C
% 半径R
% 法向量normal

% 定义角度向量（这里是绘制整个圆，所以从0到2*pi）
t = 0:0.01:2*pi;
% 生成环形弧线的坐标
coordinates = arc(C, R, normal, t);
% 绘制环形弧线
plot3(coordinates(1,:), coordinates(2,:), coordinates(3,:), 'b', 'LineWidth', 2);

end

