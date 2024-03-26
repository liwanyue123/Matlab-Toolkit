% 定义贝塞尔曲线的控制点
pos_start = [0; 0; 0];
pos_end = [0.3; 0.3; 0];
height = 0.2;

p0 = pos_start;   % 初始位置
p1 = pos_start;   % 控制点1
p2 = pos_start;   % 控制点2
p3 = pos_end;     % 控制点3
p4 = pos_end;     % 最终位置

p2(3) = p2(3) + height;


vel=[2;0;0];
kBezierDegree=4;
dalpha_dt=1/0.4;

bezier_delta = [1; 1; 1];
bezier_delta(1)=vel(1)/dalpha_dt/kBezierDegree;
bezier_delta(2)=vel(2)/dalpha_dt/kBezierDegree;
bezier_delta(3)=vel(3)/dalpha_dt/kBezierDegree;

p1(1:2) = p1(1:2) + bezier_delta(1:2);
p2 = p2 - bezier_delta;
p3 = p3 - bezier_delta;

% 设置曲线的步长
step = 0.05;

% 计算贝塞尔曲线上的点
points = [];
for t = 0:step:1
    point = (1-t)^4 * p0 + 4*(1-t)^3*t * p1 + 6*(1-t)^2*t^2 * p2 + 4*(1-t)*t^3 * p3 + t^4 * p4;
    points = [points point];
end

% 绘制贝塞尔曲线和控制点
% figure;
plot3(points(1,:), points(2,:), points(3,:), 'LineWidth', 2);
hold on;
scatter3([p0(1), p1(1), p2(1), p3(1), p4(1)], [p0(2), p1(2), p2(2), p3(2), p4(2)], [p0(3), p1(3), p2(3), p3(3), p4(3)], 'r', 'filled');
for i = 1:5
    x = [p0(1), p1(1), p2(1), p3(1), p4(1)];
    y = [p0(2), p1(2), p2(2), p3(2), p4(2)];
    z = [p0(3), p1(3), p2(3), p3(3), p4(3)];
    text(x(i), y(i), z(i), num2str(i));
end
title('Simple Bezier Curve Visualization');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
legend('Bezier Curve', 'Control Points');
