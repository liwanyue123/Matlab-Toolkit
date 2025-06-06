addpath(genpath("TwistCalculation/twistAndScrew"));
addpath(genpath("TwistCalculation/drawPic"));
addpath(genpath("Coordinate"));
addpath(genpath("Matrix"));
addpath(genpath("Utils"));
addpath(genpath("simple"));
addpath(genpath("SphereTools"));

function [lineList, T_cum] = drawLegJoint(leg_struct, T_base, C_world, labelPrefix, color)
    T_cum = T_base;
    lineList = [];
    lineList = [lineList, getCoordDotFromT(T_base)];  % 初始坐标点
   
    n = length(leg_struct);
       % drawLegJoint函数中
    for i = 1:n
        T_joint = calTranMatrixFromR(leg_struct(i).type, leg_struct(i).theta);
        % base -> 1 -> 2 -> 3 -> foot
        % T_base_1= T_base * joint1, 
        % T_base_2= T_base_1* T_1_2 * joint2, 
        % T_base_3= T_base_2* T_2_3 * joint3, 
        % T_base_foot= T_base_3* T_3_foot * joint_foot, 
        T_cum = T_cum * leg_struct(i).T * T_joint;  % 矩阵乘法顺序修改
        Ci = genCoordinateCoord(C_world, T_cum);
        showCoordinate(Ci, sprintf(leg_struct(i).name, labelPrefix, i), 0.1);
        lineList = [lineList, Ci.p0(:)];
    end

    x_real = T_cum(1:3,4);
    % 绘制连接线
    showLine(lineList, '-', color, 2);
end

% 从 base 累加变换，算出 base frame 下的 J_base；
% 再通过 adjoint 变换，把整个 Jacobian 映射到末端坐标系。
function [x, J_foot] = forwardKinematics(leg_struct)
    T_cum = eye(4);
    J_base = [];

    for i = 1:length(leg_struct)
        T_joint = calTranMatrixFromR(leg_struct(i).type, leg_struct(i).theta);
        T_cum = T_cum * leg_struct(i).T * T_joint;

        X = calAdjointMatFromT(T_cum, 'V');  % base -> i
        S_i = generateNormTwsit(leg_struct(i).type, 'Rotational');
        J_base = [J_base, X * S_i];
    end

    x = T_cum(1:3, 4);  % 足端位置

    % 足端坐标系下的 Jacobian
    Ad_inv = calAdjointMatFromT(inv(T_cum), 'V');
    J_foot = Ad_inv * J_base;

    % % world系
    % F_w=generateNormTwsit('Z', 'Prismatic')*10
    % % 转成末端坐标系   
    % X_foot_base=Ad_inv ;
    % F_f=X_foot_base*F_w
    % 
    % tau=transpose(J_foot)*F_f % 转的也是关节局部坐标系
end

function [theta_sol, success, theta_history, error_history] = numericIK(leg_struct, x_desired, max_iter, tol, leg_side)
    if nargin < 3
        max_iter = 500;
    end
    if nargin < 4
        tol = 1e-2;
    end
    if nargin < 5
        leg_side = 'left'; % 默认使用左腿
    end

    % 使用初始化猜想而非全零初始化
    initial_guess = get_initial_guess(leg_side);
    for i = 1:length(leg_struct)
        leg_struct(i).theta = initial_guess(i);
    end
    
    theta = [leg_struct.theta]';
    theta_history = theta';
    error_history = []; 
    
    % 获取关节限制
    joint_limits = get_joint_limits(leg_side);
    
    % 计算初始误差
    [x_init, ~] = forwardKinematics(leg_struct);
    initial_error = norm(x_desired - x_init);
    error_history = [error_history, initial_error];
    fprintf('[IK] Initial error: %.5f | Using initial guess\n', initial_error);

    % 改进参数配置
    lambda = 0.1;
    min_lambda = 0.01;
    max_lambda = 1.0;
    
    learning_rate = 2.0;
    min_learning_rate = 0.4;
    reduction_factor = 0.8;
    
    prev_error = initial_error;
    
    % 添加最佳解记录机制
    best_theta = theta;
    best_error = initial_error;
    best_iter = 1;
    
    for iter = 1:max_iter
        for i = 1:length(leg_struct)
            leg_struct(i).theta = theta(i);
        end

        [x, J_foot] = forwardKinematics(leg_struct);
        e = x_desired - x;
        current_error = norm(e);
        error_history = [error_history, current_error];

        % 更新最佳解
        if current_error < best_error
            best_theta = theta;
            best_error = current_error;
            best_iter = iter;
        end
        
        % 收敛检查 (使用更严格的阈值)
        if current_error < tol
            success = true;
            theta_sol = theta;
            fprintf('[IK] Converged at iter %d, error = %.5f\n', iter, current_error);
            return;
        end
        
        J = J_foot(4:6, :);
        
        % 自适应阻尼和学习率控制
        error_ratio = current_error / prev_error;
        
        if error_ratio > 0.95 % 收敛困难
            lambda = min(lambda * 1.2, max_lambda);
            learning_rate = max(learning_rate * 0.85, min_learning_rate);
        elseif error_ratio < 0.7 % 良好收敛
            lambda = max(lambda * 0.8, min_lambda);
            learning_rate = min(learning_rate * 1.1, 1.0);
        end
        
        % 使用阻尼伪逆解算增量
        JtJ = J' * J;
        delta_theta = (JtJ + lambda * eye(size(JtJ))) \ (J' * e);
        
        % 应用学习率
        delta_theta = learning_rate * delta_theta;
        
        % 自适应步长控制
        max_step = min(0.3, 0.1 + 0.1 * current_error);
        norm_delta = norm(delta_theta);
        if norm_delta > max_step
            delta_theta = (max_step / norm_delta) * delta_theta;
            fprintf('[IK] Iter %d: Step clipped to %.4f rad\n', iter, max_step);
        end
        
        % 更新关节角
        theta = theta + delta_theta;
        
        % 应用详细的关节限制
        for i = 1:length(leg_struct)
            theta(i) = max(joint_limits(i, 1), min(joint_limits(i, 2), theta(i)));
        end
 
        % 每10次迭代重置到最佳解（防止发散）
        if mod(iter, 10) == 0 && best_error < prev_error && iter > best_iter + 2
            fprintf('[IK] Reset to best solution (error %.5f at iter %d)\n', best_error, best_iter);
            theta = best_theta;
            prev_error = best_error;
        else
            % 记录并更新误差
            theta_history = [theta_history; theta'];
            prev_error = current_error;
        end

        % 如果超过20次迭代没有改进，提前终止
        if iter - best_iter > 20
            warning('[IK] No improvement in last 20 iterations. Stopping.');
            theta = best_theta;
            break;
        end
    end

    if best_error < tol * 2
        success = true;
        theta_sol = best_theta;
        fprintf('[IK] Partial success. Final error = %.5f\n', best_error);
    else
        warning('[IK] Not converged. Best error = %.5f', best_error);
        theta_sol = best_theta;
        success = false;
    end
end

% 关节限制功能
function limits = get_joint_limits(leg_side)
    % 根据不同关节类型和腿侧返回限制
    switch lower(leg_side)
        case 'left'
            limits = [
                -2.5307,  2.8798;  % hip_pitch
                -0.5236,  2.9671;  % hip_roll
                -2.7576,  2.7576;  % hip_yaw
                -0.0873,  2.8798;  % knee
                -0.8727,  0.5236;  % ankle_pitch
                -0.2618,  0.2618;  % ankle_roll
            ];
        case 'right'
            limits = [
                -2.5307,  2.8798;  % hip_pitch
                -2.9671,  0.5236;  % hip_roll
                -2.7576,  2.7576;  % hip_yaw
                -0.0873,  2.8798;  % knee
                -0.8727,  0.5236;  % ankle_pitch
                -0.2618,  0.2618;  % ankle_roll
            ];
        otherwise
            error('Invalid leg side. Use "left" or "right".');
    end
end

% 初始化猜想功能
function guess = get_initial_guess(leg_side)
    % 根据腿侧返回合理的初始关节角
    switch lower(leg_side)
        case 'left'
            % 左腿初始猜测
            guess = [-0.2, -0.1, 0.02, 0.6, 0.05, 0.0];
        case 'right'
            % 右腿初始猜测
            guess = [-0.2, 0.1, -0.02, 0.6, 0.05, 0.0];
        otherwise
            % 默认初始值
            guess = zeros(1, 6);
    end
end

function animateIK(theta_history, leg_struct, T_base, C_world, error_history, x_target)
    % 创建单个窗口，左右分屏布局
    figure('Name', 'IK Visualization', 'Position', [100, 100, 1200, 600]);
    
    % 左子图：3D动画
    subplot(1,2,1);
    ax1 = gca;
    axis equal;
    grid on;
    view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis([-0.4, 0.4, -0.4, 0.4, -0.6, 0.1]); % 固定坐标轴范围
    
    % 右子图：误差曲线
    subplot(1,2,2);
    ax2 = gca;
    plot(0:length(error_history)-1, error_history, 'b-o', 'LineWidth', 2);
    grid on;
    xlabel('Iteration');
    ylabel('Error Norm (m)');
    title('IK Convergence Performance');
    hold on;
    
    % 为右图添加标记当前点的初始点（空白）
    current_point = plot(0, error_history(1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    text(0.5, max(error_history)*0.9, sprintf('Current Error: %.4f', error_history(1)), ...
        'FontSize', 12, 'Color', 'r');
    hold off;
    
    for k = 1:size(theta_history, 1)
        % 设置关节角
        for i = 1:length(leg_struct)
            leg_struct(i).theta = theta_history(k, i);
        end
        
        % === 更新左子图（动画） ===
        axes(ax1);  % 激活左子图
        cla(ax1);  % 只清除左子图内容
        
        % 在左子图绘制当前构型
        drawLegJoint(leg_struct, T_base, C_world, 'L', 'b');
        
        % 添加目标点
        hold on;
        plot3(x_target(1), x_target(2), x_target(3), 'r*', 'MarkerSize', 15);
        text(x_target(1)+0.02, x_target(2), x_target(3), 'Target', 'Color', 'r');
        
        % 计算并添加当前足端位置
        [x_current, ~] = forwardKinematics(leg_struct);
        plot3(x_current(1), x_current(2), x_current(3), 'go', 'MarkerSize', 10);
        text(x_current(1)+0.02, x_current(2), x_current(3), 'Current', 'Color', 'g');
        hold off;
        
        % 更新标题显示当前信息
        title(ax1, sprintf("Iteration %d/%d | Error: %.4f m", k, size(theta_history,1), error_history(k)));
        
        % === 更新右子图（误差曲线） ===
        axes(ax2);  % 激活右子图
        
        % 更新当前点位置
        set(current_point, 'XData', k-1, 'YData', error_history(k));
        
        % 更新文本
        title(ax2, sprintf('IK Convergence | Current Error: %.4f m', error_history(k)));
        
        % === 添加暂停 ===
        pause(0.1);  % 控制帧率
    end
    
    % 添加总执行结果
    if error_history(end) < 0.002
        final_status = 'Converged';
        color = 'g';
    else
        final_status = 'Failed to converge';
        color = 'r';
    end
    
    annotation_text = sprintf('%s | Final Error: %.4f m', final_status, error_history(end));
    text(ax2, size(theta_history,1)*0.1, min(error_history)*1.5, annotation_text, ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', color);
end

% 腿模型定义
% foot 三个分支变换矩阵（共用）
T_foot_front = [1 0 0 0.180; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot_mid   = [1 0 0 0.000; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot_hind  = [1 0 0 -0.090; 0 1 0 0; 0 0 1 -0.060; 0 0 0 1];
T_foot = {T_foot_front, T_foot_mid, T_foot_hind};

% 左腿
leg_left = struct(...
    'type', {'Y','X','Z','Y','Y','X'}, ...
    'theta', {0, 0, 0, 0, 0, 0}, ...
    'name', {'hip_{pitch}','hip_{roll}','hip_{yaw}','knee','ankle_{pitch}','ankle_{roll}'}, ...
    'T', {...
        eye(4),...
        [ 0.984744 0 -0.174010 0; 0 1 0 0.052000; 0.174010 0 0.984744 -0.030465; 0 0 0 1],...
        [ 1 0 0 0.025001; 0 1 0 0; 0 0 1 -0.124120; 0 0 0 1],...
        [ 0.984744 0 0.174010 -0.078273; 0 1 0 0.002149; -0.174010 0 0.984744 -0.177340; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 -0.000094; 0 0 1 -0.300010; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 0; 0 0 1 -0.017558; 0 0 0 1],...
    } ...
);
T_base_leg_left =[ 1 0 0 0; 0 1 0 0.064452; 0 0 1 -0.102700; 0 0 0 1];

% 右腿
leg_right = struct(...
    'type', {'Y','X','Z','Y','Y','X'}, ...
    'theta', {0, 0, 0, 0, 0, 0}, ...
    'name', {'hip_{pitch}','hip_{roll}','hip_{yaw}','knee','ankle_{pitch}','ankle_{roll}'}, ...
    'T', {...
        eye(4),...
        [ 0.984744 0 -0.174010 0; 0 1 0 -0.052000; 0.174010 0 0.984744 -0.030465; 0 0 0 1],...
        [ 1 0 0 0.025001; 0 1 0 0; 0 0 1 -0.124120; 0 0 0 1],...
        [ 0.984744 0 0.174010 -0.078273; 0 1 0 -0.002149; -0.174010 0 0.984744 -0.177340; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 0.000094; 0 0 1 -0.300010; 0 0 0 1],...
        [ 1 0 0 0; 0 1 0 0; 0 0 1 -0.017558; 0 0 0 1],...
    } ...
);
% T_base_leg_right =[ 1 0 0 0; 0 1 0 -0.064452; 0 0 1 -0.102700; 0 0 0 1];
 

% 设置目标足端位置
x_target = [0.2; 0.0; -0.5];
% x_target = [-0.1; 0.0; -0.4];

% 仅运行一次 IK 求解
[x, J_foot] = forwardKinematics(leg_left);
[theta_ik, success, theta_history, error_history] = numericIK(leg_left, x_target);

% 展示结果
fprintf('IK求解完成。迭代次数: %d, 最终误差: %.5f m\n', size(theta_history,1), error_history(end));

% 展示动画和误差曲线
animateIK(theta_history, leg_left, eye(4), C_world, error_history, x_target);
