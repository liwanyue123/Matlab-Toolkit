function [] = drawLineArrow(plot,vec,color,rate )
%�ڿռ�ĳ������ʾһ��������������������ϵ
 
assert(size(plot,1) == 3&& size(plot,2) == 1,"plot Must have 3x1 matrix");
assert(size(vec,1) == 3&& size(vec,2) == 1,"vec Must have 3x1 matrix");


%vararginд���� 'color','red','stemWidth',0.02,'facealpha',0.5
size_param=[0.02,0.05];
size_param=size_param*rate;
mArrow3(plot,plot+vec,'color',color,'facealpha',0.5,'stemWidth',size_param(1),'tipWidth',size_param(2));
 
   
end

