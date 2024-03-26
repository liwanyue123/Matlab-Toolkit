function [V1] = calTwistFromOtherTwist(p0,V0,r_01,showType,rate,color)

T1=calTranMatrixFromP(r_01);

Adj_T1=calAdjointMatFromT(T1^(-1),"V");
V1=Adj_T1*V0

showTwist(p0+r_01,V1,showType,rate,color);


end

