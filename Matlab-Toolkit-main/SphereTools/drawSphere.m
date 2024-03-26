function [] = drawSphere()
% 生成单位球的表面
[x,y,z] = sphere;
% 绘制单位球
surf(x, y, z, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1)
hold on

end

