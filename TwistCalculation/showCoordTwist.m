function [] = showCoordTwist(Coord,Twist,show,rate)
%��ĳ������ϵCoordԭ�㴦��ʾ��ʹ���������ϵ���׵�һ��������ע�����Ǹ�����ϵԭ�㴦����Ȼ�����临��
%show,��Ϊͬһ�������ϸ�����ٶ�һ�£�����һ�����ǲ�����ʾ��
%rate���������ȵ����ű���
%����������ϵ��pot�㴦����ʾ����Twist

w=Twist(1:3)*rate;
v=Twist(4:6)*rate;
%����Ҫ���������ϵ��ԭ�㣬����������ת����������ϵ�� 


%������������ϵ�ı�ʾ��ת����������������
pot=XYZ2p(Coord.p0)';%Ŀ������ϵ��ԭ������������ϵ�µ�λ��

%������Ϊ������ϵ���������
X_w2coord=XYZ2p(Coord.x1)'-pot;
Y_w2coord=XYZ2p(Coord.y1)'-pot;
Z_w2coord=XYZ2p(Coord.z1)'-pot;

R_w2coord=[X_w2coord,Y_w2coord,Z_w2coord];
        
%���ٶȿ����ռ���һ�㣬��������������ϵ����ʾ��
w=R_w2coord*w;
v=R_w2coord*v;

if(show=="all")
    circleArrow(pot ,w) ;
lineArrow(pot,v );
elseif(show=="w")
    circleArrow(pot ,w) ;

elseif(show=="v")
    lineArrow(pot,v ,"blue");
elseif(show=="null")
end
 
end

