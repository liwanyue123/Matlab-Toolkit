function [Adj_T] = calAdjointMatFromT(T,str_VorF)
% 这是齐次矩阵T的伴随矩阵，把4*4变成6*6
%之后就可以作用到旋量上面了，不仅仅改变基底，
% 而且假设刚体无限大，由已知坐标系原点处的旋量，推出目标坐标系原点处的旋量

R=getRfromT(T);
P=getPfromT(T);
P_x= genCrossMatrix(P);
if(str_VorF=="V")%如果是速度旋量
    Adj_T=[R,zeros(3,3);P_x*R,R];
elseif(str_VorF=="F")%如果是力旋量，这个变换矩阵和上面是对偶的
    Adj_T=[R,P_x*R;zeros(3,3),R];
end
end

