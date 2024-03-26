% 定义圆心
C = [0; 0; 0];

% 定义半径
R = 1;

% 定义法向量
normal = [1; 1; 1];


hold on
% 绘制环形弧线
drawArc(C,R, normal)
drawVector(C, normal)

[x, y] = meshgrid(-range:distance:range, -range:distance:range);
z = 0 * ones(size(x));
surf(x, y, z, 'FaceAlpha', 0.5);
% surf(z, y, x, 'FaceAlpha', 0.5);
% surf(x, z, y, 'FaceAlpha', 0.5);

axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Circular Arc');
grid on;
