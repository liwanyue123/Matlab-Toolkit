function [] = circleArrow(center,norm)

% center = [0 0 0];%圆心坐标
% norm = [1 0 0];%法向量
 
start_angle = pi/6;
end_angle = pi/6+pi;

rad = 0.5;%圆圈半径
circLineWidth = 2;%圆圈线粗细

len = 0.2;%箭头长度,这个才是关键
tipWidth = 0.1;
tipLength = 1;
stemWidth = 1;

 
 % mArrow3(center,center+0.4*norm,'color',[0 0 1],'stemWidth',0.01,'tipWidth',0.02);
vecarrow(start_angle,end_angle,center,norm,rad,len,tipWidth,tipLength,stemWidth,circLineWidth);
end

