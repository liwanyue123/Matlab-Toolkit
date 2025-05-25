function [] = showTwist(pot,Twist ,show,rate,color)
% varargin写的是 'color','red','stemWidth',0.02,'facealpha',0.5
%show,因为同一个刚体上各点角速度一致，所以一般我们不想显示它
%rate是向量长度的缩放比例
%在世界坐标系的pot点处，显示旋量Twist
w=Twist(1:3)*rate;
v=Twist(4:6)*rate;
 
if(show=="all")
    circleArrow(pot ,w) ;
lineArrow(pot,v ,color );
elseif(show=="w")
    circleArrow(pot ,w) ;

elseif(show=="v")
    lineArrow(pot,v ,color);
elseif(show=="null")
end
 
end

