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

    x_real = T_cum(1:3,4)
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
function [theta_sol, success, theta_history] = numericIK(leg_struct, x_desired, max_iter, tol)
    if nargin < 3
        max_iter = 500;
    end
    if nargin < 4
        tol = 2e-3;
    end

    theta = [leg_struct.theta]';  % 初始关节角列向量
    theta_history = theta';       % 每一行是一次迭代的 theta

    for iter = 1:max_iter
        for i = 1:length(leg_struct)
            leg_struct(i).theta = theta(i);
        end

        [x, J_foot] = forwardKinematics(leg_struct);
        e = x_desired - x;

        if norm(e) < tol
            success = true;
            theta_sol = theta;
            fprintf('[IK] Converged at iter %d, error = %.5f\n', iter, norm(e));
            return;
        end

        % delta_theta = pinv(J_foot(4:6, :)) * e;
        lambda = 0.6;  % 阻尼系数
        J = J_foot(4:6, :);
        delta_theta = (J' * J + lambda * eye(size(J,2))) \ (J' * e);

        delta_theta = max(min(delta_theta, 1), -1);

        theta = theta + delta_theta;
        theta_history = [theta_history; theta'];  % 添加历史记录
    end

    warning('[IK] Not converged. Final error = %.5f', norm(e));
    theta_sol = theta;
    success = false;
end

function animateIK(theta_history, leg_struct, T_base, C_world)
    figure;
    axis equal;
    grid on;
    view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-0.5, 0.5]); ylim([-0.5, 0.5]); zlim([-1, 0.2]);

    for k = 1:size(theta_history, 1)
        % 设置关节角
        for i = 1:length(leg_struct)
            leg_struct(i).theta = theta_history(k, i);
        end

        cla;  % 清除上一帧图像
        drawLegJoint(leg_struct, T_base, C_world, 'L', 'b');

        title(sprintf("Iteration %d", k));
        pause(0.1);  % 控制帧率
    end
end

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
 

% 假设目标足端位置是 [0.1; 0.0; -0.5]
x_target = [-0.2; 0.0; -0.4];
 
 
% 运行 IK 求解
[x, J_foot] = forwardKinematics(leg_left);
[theta_ik, success, theta_history] = numericIK(leg_left, x_target);

% 世界坐标系初始化
C_world = showWorldCoordinate(0.1);
% 展示动画
if success
    % ...
    animateIK(theta_history, leg_left, eye(4), C_world);
else
    warning("IK 求解失败，仍展示过程");
    animateIK(theta_history, leg_left, eye(4), C_world);
end





% 左腿主链
% [lineList_left, T_left_foot]  = drawLegJoint(leg_left, T_base_leg_left, C_world, 'L', 'b');
% [lineList_left, T_left_foot]  = drawLegJoint(leg_left, eye(4), C_world, 'L', 'b');

% % 左脚三趾
% for i = 1:length(T_foot)
%     [~, ~] = drawLeg({T_foot{i}}, T_left_foot, C_world, sprintf('L_{f%d}', i), 'b');
% end

% % 右腿主链
% [lineList_right, T_right_foot]= drawLegJoint(leg_right, T_base_leg_right, C_world, 'R', 'r');

% % 右脚三趾
% for i = 1:length(T_foot)
%     [~, ~] = drawLeg({T_foot{i}}, T_right_foot, C_world, sprintf('R_{f%d}', i), 'r');
% end


% 美化显示
% figure('Position', [100, 100, 800, 600]);  % 左下角(x,y), 宽, 高
view(37.5, 30);   % 默认 3D 视角
axis equal;
title('Left and Right Leg Kinematic Chain Visualization');
