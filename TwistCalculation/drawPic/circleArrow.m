function [] = circleArrow(center,norm)

% center = [0 0 0];%Բ������
% norm = [1 0 0];%������
 
start_angle = pi/6;
end_angle = pi/6+pi;

rad = 0.5;%ԲȦ�뾶
circLineWidth = 2;%ԲȦ�ߴ�ϸ

len = 0.2;%��ͷ����,������ǹؼ�
tipWidth = 0.1;
tipLength = 1;
stemWidth = 1;

 
 % mArrow3(center,center+0.4*norm,'color',[0 0 1],'stemWidth',0.01,'tipWidth',0.02);
vecarrow(start_angle,end_angle,center,norm,rad,len,tipWidth,tipLength,stemWidth,circLineWidth);
end

