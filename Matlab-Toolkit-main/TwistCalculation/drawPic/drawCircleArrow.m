function [] = drawCircleArrow(pos,tau,R,norm_max_tau,rate,color)

% pos = [0 0 0];%Բ������
% tau = [1 0 0];%������
 
% Ϊ�˸���Ť�ش�С���ӻ������ٷ�֮���ٵ�Բ
persent_rad=norm(tau)/norm_max_tau;


% R = 0.5;%ԲȦ�뾶

% len = 0.2;%��ͷ����,������ǹؼ�
% tipWidth = 0.1;
% tipLength = 1;
% stemWidth = 1;
size_param=[0.2,0.1,1,1];
size_param=size_param*rate;
drawArcArrow(pos,tau,R,persent_rad,size_param(1),size_param(2),size_param(3),size_param(4),color);
end

