function [] = drawArcArrow( C, normal, R, persent_rad, len, tipWidth, tipLength, stemWidth ,color)
% draw带箭头的圆弧
% 圆心C
% 半径R
% 法向量normal
% 画百分之多少的圆 persent_rad [0,1]

if(persent_rad==0)
    return
end
if(norm(normal)==0)
    return
end

norm_vec = normal/ norm(normal);
% 定义角度向量（这里是绘制整个圆，所以从0到2*pi）
t = 0:0.01:2*persent_rad*pi;

% 生成环形弧线的坐标
coordinates = arc(C, R, norm_vec, t);
% 绘制环形弧线
plot3(coordinates(1,:), coordinates(2,:), coordinates(3,:), color, 'LineWidth', 2);

X=coordinates(1,:);
Y=coordinates(2,:);
Z=coordinates(3,:);

% 计算箭头的方向向量
tgt = coordinates(:, end) - coordinates(:, end - 1);

% 绘制箭头
mArrow3(coordinates(:, end), coordinates(:, end) + len * tgt / norm(tgt), 'stemWidth', stemWidth, 'tipWidth', tipWidth, 'tipLength', tipLength);

end




