function [] = showTwist(pot,Twist ,show,rate,color)
% vararginд���� 'color','red','stemWidth',0.02,'facealpha',0.5
%show,��Ϊͬһ�������ϸ�����ٶ�һ�£�����һ�����ǲ�����ʾ��
%rate���������ȵ����ű���
%����������ϵ��pot�㴦����ʾ����Twist
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

