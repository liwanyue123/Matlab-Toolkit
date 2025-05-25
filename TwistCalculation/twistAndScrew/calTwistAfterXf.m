function [V_b] = calTwistAfterXf(V_a,Tran_ab)
%将旋量进行旋量变换，得到一个新旋量，这个旋量的坐标系基底也变成了新的

%因为 V_b=X_ba*V_a，所以Tran_ab要取逆成X_ba
if(size(Tran_ab,1) == 6 && size(Tran_ab,2) == 6)%直接就是旋量变换矩阵
    %你给的这个Tran_ab一定要是先平移后旋转呀！！！！1
    X_ba=invertSXform_pos(Tran_ab);
    
elseif(size(Tran_ab,1) == 4 && size(Tran_ab,2) == 4)%齐次变换矩阵，要先转成旋量变换矩阵
     X_ba=calAdjointMatFromT(Tran_ab^(-1),"V");
     
 elseif(size(Tran_ab,1) == 3 && size(Tran_ab,2) == 3)   %旋转矩阵
     T=composeTfromRandP(Tran_ab',zeros(3,1));   %先构建齐次变换矩阵
     X_ba=calAdjointMatFromT(T,"V");%转成旋量变换矩阵
     
 elseif(size(Tran_ab,1) == 3 && size(Tran_ab,2) == 1)   %平移
     T=composeTfromRandP(eye(3),-Tran_ab);   %先构建齐次变换矩阵
     X_ba=calAdjointMatFromT(T,"V");%转成旋量变换矩阵
else
    error("没见过这种形状的变换矩阵");
end

V_b=X_ba*V_a;

 

end

