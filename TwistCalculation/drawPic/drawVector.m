function drawVector(start_point, vector)
    % start_point: 起点的坐标，形如 [x, y, z]
    % vector: 向量，形如 [x_component, y_component, z_component]
    
    % 终点坐标等于起点坐标加上向量
    end_point = start_point + vector;
    
    % 绘制虚线向量
    line([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], 'Color', 'k', 'LineStyle', '--');
    
    % 绘制起点
    hold on;
    plot3(start_point(1), start_point(2), start_point(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.5, 0.5], 'MarkerEdgeColor', [0.5, 0.5, 0.5]);
    
    % 绘制终点
    plot3(end_point(1), end_point(2), end_point(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.5, 0.5], 'MarkerEdgeColor', [0.5, 0.5, 0.5]);
    
end
