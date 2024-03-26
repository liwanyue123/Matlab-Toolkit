function [F_B] = calScrewFromOtherScrew(F_A,r_BA)

T1=calTranMatrixFromP(r_BA);

Adj_T1=calAdjointMatFromT(T1,"F");
F_B=Adj_T1*F_A

% showTwist(p0+r_BA,F_B,showType,rate,color);


end

