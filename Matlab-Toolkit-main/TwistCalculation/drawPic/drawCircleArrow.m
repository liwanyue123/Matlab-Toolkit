function [] = drawCircleArrow(pos,tau,norm_max_tau,color)

% pos = [0 0 0];%Բ������
% tau = [1 0 0];%������
 
% Ϊ�˸���Ť�ش�С���ӻ������ٷ�֮���ٵ�Բ
persent_rad=norm(tau)/norm_max_tau;


R = 0.5;%ԲȦ�뾶

len = 0.2;%��ͷ����,������ǹؼ�
tipWidth = 0.1;
tipLength = 1;
stemWidth = 1;

drawArcArrow(pos,tau,R,persent_rad,len,tipWidth,tipLength,stemWidth,color);
end

