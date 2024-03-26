function [] = showTwist(pot,Twist ,show,rate,color)
%在世界坐标系的pot点处，显示旋量Twist
%rate是向量长度的缩放比例
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

