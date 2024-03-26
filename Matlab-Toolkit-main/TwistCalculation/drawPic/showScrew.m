function [] = showScrew(pot,Twist,show,rate,color,norm_max_tau)
%在世界坐标系的pot点处，显示旋量Twist
%rate是向量长度的缩放比例

t=Twist(1:3)*rate;
f=Twist(4:6)*rate;

if(show=="all")
    drawCircleArrow(pot ,t,norm_max_tau,color) ;
    drawLineArrow(pot,f ,color);
elseif(show=="t")
    drawCircleArrow(pot ,t,norm_max_tau,color) ;

elseif(show=="f")
    drawLineArrow(pot,f ,color);
elseif(show=="null")
end

end

