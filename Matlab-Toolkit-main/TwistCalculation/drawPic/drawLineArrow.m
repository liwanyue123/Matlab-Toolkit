function [] = drawLineArrow(plot,vec,color )
%在空间某点上显示一个向量，都是世界坐标系
 
assert(size(plot,1) == 3&& size(plot,2) == 1,"plot Must have 3x1 matrix");
assert(size(vec,1) == 3&& size(vec,2) == 1,"vec Must have 3x1 matrix");


%varargin写的是 'color','red','stemWidth',0.02,'facealpha',0.5
 mArrow3(plot,plot+vec,'color',color,'facealpha',0.5,'stemWidth',0.02,'tipWidth',0.05);
 
   
end

