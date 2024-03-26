function [ ] = show_w(w)
% 绘制旋转后的向量
quiver3(0,0,0, w(1), w(2), w(3), 'LineWidth', 2, 'Color', 'r')


end

