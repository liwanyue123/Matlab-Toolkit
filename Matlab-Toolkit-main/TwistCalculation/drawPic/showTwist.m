function [] = showTwist(pot,Twist ,show,rate,color)
%����������ϵ��pot�㴦����ʾ����Twist
%rate���������ȵ����ű���
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

