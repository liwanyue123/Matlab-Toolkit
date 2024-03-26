function [] = drawCircleArrow(pos,tau,R,norm_max_tau,rate,color)

% pos = [0 0 0];%圆心坐标
% tau = [1 0 0];%法向量
 
% 为了根据扭矩大小可视化，画百分之多少的圆
persent_rad=norm(tau)/norm_max_tau;


% R = 0.5;%圆圈半径

% len = 0.2;%箭头长度,这个才是关键
% tipWidth = 0.1;
% tipLength = 1;
% stemWidth = 1;
size_param=[0.2,0.1,1,1];
size_param=size_param*rate;
drawArcArrow(pos,tau,R,persent_rad,size_param(1),size_param(2),size_param(3),size_param(4),color);
end

