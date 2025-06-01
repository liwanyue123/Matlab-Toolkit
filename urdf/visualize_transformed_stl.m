function [h_patch, h_axes] = visualize_transformed_stl(stl_filepath, T, faceAlpha, target_axes)
    % VISUALIZE_TRANSFORMED_STL 专业STL可视化工具（不自动创建图形窗口）
    %
    % 输入参数:
    %   stl_filepath - STL文件路径
    %   T - 4x4齐次变换矩阵（可选，默认eye(4)）
    %   faceAlpha - 透明度0-1（可选，默认0.7）
    %   target_axes - 目标坐标轴句柄（可选）
    %
    % 输出参数:
    %   h_patch - 创建的patch对象句柄
    %   h_axes - 使用的坐标轴句柄
    %
    % 使用示例:
    %   % 在现有图形中可视化
    %   figure; ax = axes;
    %   visualize_transformed_stl('part.stl', eye(4), 0.8, ax);
    %
    %   % 作为组件使用
    %   h = visualize_transformed_stl('arm.stl', makehgtform('translate',[1 0 0]));

    % ========== 参数验证 ==========
    if nargin < 2, T = eye(4); end
    if nargin < 3, faceAlpha = 0.7; end
    validateattributes(T, {'numeric'}, {'size',[4 4]}, mfilename, 'T');
    validateattributes(faceAlpha, {'numeric'}, {'scalar','>=',0,'<=',1}, mfilename, 'faceAlpha');

    % ========== 坐标轴处理 ==========
    if nargin < 4 || isempty(target_axes)
        if isempty(get(groot,'CurrentFigure')) || isempty(get(gcf,'CurrentAxes'))
            h_axes = axes('NextPlot','add');
        else
            h_axes = gca;
            hold(h_axes,'on');
        end
    else
        h_axes = target_axes;
        hold(h_axes,'on');
    end

    % ========== 模型加载与变换 ==========
    try
        [faces, vertices] = stlread(stl_filepath);
    catch ME
        error('STL读取失败: %s', ME.message);
    end
    vertices = (T(1:3,1:3) * vertices' + T(1:3,4))';

    % ========== 专业渲染配置 ==========
    h_patch = patch(h_axes, 'Faces',faces, 'Vertices',vertices,...
                   'FaceColor',[0.55 0.55 0.55],...  % 中灰色
                   'EdgeColor','none',...
                   'FaceAlpha',faceAlpha,...
                   'DiffuseStrength',0.6,...
                   'SpecularStrength',0.25,...
                   'AmbientStrength',0.45,...
                   'BackFaceLighting','lit');

    % ========== 智能光照管理 ==========
    existing_lights = findobj(h_axes,'Type','light');
    if isempty(existing_lights)
        % 主光源（45度侧光）
        light(h_axes, 'Position',[1 1 0.7], 'Style','infinite',...
              'Color',[0.85 0.85 0.8]);
        
        % 补光（背光）
        light(h_axes, 'Position',[-0.5 -1 0.3], 'Style','infinite',...
              'Color',[0.3 0.3 0.5]);
        
        % 环境光（顶部）
        light(h_axes, 'Position',[0 0 1], 'Style','local',...
              'Color',[0.4 0.4 0.4]);
    end

    % ========== 材质配置 ==========
    material(h_axes, [0.35, 0.65, 0.2, 8, 0.4]);  % 类橡胶材质

    % ========== 坐标系标注 ==========
    if ~hold_state
        % 仅当坐标轴为空时绘制参考坐标系
        % 世界坐标系
        plot3(h_axes, [0 0.1], [0 0], [0 0], 'r-', 'LineWidth', 1.5);
        plot3(h_axes, [0 0], [0 0.1], [0 0], 'g-', 'LineWidth', 1.5);
        plot3(h_axes, [0 0], [0 0], [0 0.1], 'b-', 'LineWidth', 1.5);
        
        % 局部坐标系
        origin = T(1:3,4)';
        axis_len = 0.15;
        plot3(h_axes, [origin(1), origin(1)+T(1,1)*axis_len],...
                    [origin(2), origin(2)+T(2,1)*axis_len],...
                    [origin(3), origin(3)+T(3,1)*axis_len],...
                    'r--', 'LineWidth', 1.2);
        plot3(h_axes, [origin(1), origin(1)+T(1,2)*axis_len],...
                    [origin(2), origin(2)+T(2,2)*axis_len],...
                    [origin(3), origin(3)+T(3,2)*axis_len],...
                    'g--', 'LineWidth', 1.2);
        plot3(h_axes, [origin(1), origin(1)+T(1,3)*axis_len],...
                    [origin(2), origin(2)+T(2,3)*axis_len],...
                    [origin(3), origin(3)+T(3,3)*axis_len],...
                    'b--', 'LineWidth', 1.2);
    end

    % ========== 自动视图调整 ==========
    if ~hold_state
        axis(h_axes, 'equal', 'tight');
        grid(h_axes, 'on');
        view(h_axes, 30, 30);
    end

    % ========== 输出处理 ==========
    if nargout == 0
        clear h_patch h_axes
    end

    % 辅助函数
    function state = hold_state()
        state = ishold(h_axes);
    end
end