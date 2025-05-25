function [] = showCoordinate(C, name, rate, varargin)
% showCoordinate - 显示坐标系箭头和标签，带缩放比例
% C.p0 是起点，C.x1/y1/z1 是三个轴的终点，rate 控制箭头和标签长度比例

hold on

% 箭头方向向量
x_vec = [(C.x1.X - C.p0.X), (C.x1.Y - C.p0.Y), (C.x1.Z - C.p0.Z)];
y_vec = [(C.y1.X - C.p0.X), (C.y1.Y - C.p0.Y), (C.y1.Z - C.p0.Z)];
z_vec = [(C.z1.X - C.p0.X), (C.z1.Y - C.p0.Y), (C.z1.Z - C.p0.Z)];

% 缩放后终点位置，用于放置文字标签
x_end = [C.p0.X, C.p0.Y, C.p0.Z] + rate * x_vec;
y_end = [C.p0.X, C.p0.Y, C.p0.Z] + rate * y_vec;
z_end = [C.p0.X, C.p0.Y, C.p0.Z] + rate * z_vec;

% 默认颜色处理
if nargin < 4
    color_x = 'r'; color_y = 'g'; color_z = 'b'; label_color = 'r';
else
    color_x = varargin{1}; color_y = varargin{1}; color_z = varargin{1}; label_color = varargin{1};
end

% 坐标系名称
text(C.p0.X, C.p0.Y, C.p0.Z, name, 'color', label_color, 'FontSize', 14);

% 画箭头
quiver3(C.p0.X, C.p0.Y, C.p0.Z, x_vec(1), x_vec(2), x_vec(3), rate, 'Color', color_x);
quiver3(C.p0.X, C.p0.Y, C.p0.Z, y_vec(1), y_vec(2), y_vec(3), rate, 'Color', color_y);
quiver3(C.p0.X, C.p0.Y, C.p0.Z, z_vec(1), z_vec(2), z_vec(3), rate, 'Color', color_z);

% 坐标轴标签
text(x_end(1), x_end(2), x_end(3), 'x', 'Color', color_x, 'FontSize', 14);
text(y_end(1), y_end(2), y_end(3), 'y', 'Color', color_y, 'FontSize', 14);
text(z_end(1), z_end(2), z_end(3), 'z', 'Color', color_z, 'FontSize', 14);
end
