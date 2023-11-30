function [V1] = calTwistFromOtherTwist(p0,V0,r_01,showType,rate,color)
%r_01������B�������A���λ�ã�����AB����,Ҳ����������ϵ�Ļ���
%V0������A�������
%p0:��������ϵ��V0������λ��
T1=calTranMatrixFromP(r_01);%����ֻ����ƽ�ƣ�����ǰ������������ͬһ������
%����ξ���T�İ�����󣬰�4*4���6*6
Adj_T1=calAdjointMatFromT(T1^(-1),"V");%��Ȼ�һ�»���ԭ��Ӧ��ȡ��Ŷ�
V1=Adj_T1*V0;%�������ֱ�����õ��ٶ���������
 
    showTwist(p0+r_01,V1,showType,rate,color);
 

end

