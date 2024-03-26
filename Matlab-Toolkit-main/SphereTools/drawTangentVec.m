function [] = drawTangentVec(R,scale)
% 计算初始向量经过旋转矩阵 R 后的向量
v_x = R(1:3,1);
v_z = R(1:3,3);

% 绘制旋转后的向量
quiver3(v_z(1), v_z(2), v_z(3), v_x(1)*scale, v_x(2)*scale, v_x(3)*scale, 'LineWidth', 2, 'Color', 'b')





end

