function [] = showScrew(pot,Twist,show,rate,color,norm_max_tau,name)
%在世界坐标系的pot点处，显示旋量Twist
%rate是向量长度的缩放比例
R=0.5*rate;
t=Twist(1:3);
f=Twist(4:6)*rate;

if(show=="all")
    drawCircleArrow(pot ,t,R,norm_max_tau,rate,color) ;
    drawLineArrow(pot,f ,color,rate);
elseif(show=="t")
    drawCircleArrow(pot ,t,R,norm_max_tau,rate,color) ;
elseif(show=="f")
    drawLineArrow(pot,f ,color,rate);
elseif(show=="null")
end
text(pot(1),pot(2),pot(3),name,'color',color);

end

