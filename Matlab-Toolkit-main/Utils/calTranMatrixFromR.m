function [T] = calTranMatrixFromR(R)
T=eye(4);
T(1:3,1:3)=R;

end

