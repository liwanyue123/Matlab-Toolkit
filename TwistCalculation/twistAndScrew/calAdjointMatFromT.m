function [Adj_T] = calAdjointMatFromT(T,str_VorF)
% ������ξ���T�İ�����󣬰�4*4���6*6
%֮��Ϳ������õ����������ˣ��������ı���ף�
% ���Ҽ���������޴�����֪����ϵԭ�㴦���������Ƴ�Ŀ������ϵԭ�㴦������

R=getRfromT(T);
P=getPfromT(T);
P_x= genCrossMatrix(P);
if(str_VorF=="V")%������ٶ�����
    Adj_T=[R,zeros(3,3);P_x*R,R];
elseif(str_VorF=="F")%�����������������任����������Ƕ�ż��
    Adj_T=[R,P_x*R;zeros(3,3),R];
end
end

