function [] = showScrew(pot,Twist,show,rate,color,norm_max_tau)
%����������ϵ��pot�㴦����ʾ����Twist
%rate���������ȵ����ű���

t=Twist(1:3)*rate;
f=Twist(4:6)*rate;

if(show=="all")
    circleArrow(pot ,t,norm_max_tau,color) ;
    lineArrow(pot,f ,color);
elseif(show=="t")
    circleArrow(pot ,t,norm_max_tau,color) ;

elseif(show=="f")
    lineArrow(pot,f ,color);
elseif(show=="null")
end

end

