function [V_b] = calTwistAfterXf(V_a,Tran_ab)
%���������������任���õ�һ�����������������������ϵ����Ҳ������µ�

%��Ϊ V_b=X_ba*V_a������Tran_abҪȡ���X_ba
if(size(Tran_ab,1) == 6 && size(Tran_ab,2) == 6)%ֱ�Ӿ��������任����
    %��������Tran_abһ��Ҫ����ƽ�ƺ���תѽ��������1
    X_ba=invertSXform_pos(Tran_ab);
    
elseif(size(Tran_ab,1) == 4 && size(Tran_ab,2) == 4)%��α任����Ҫ��ת�������任����
     X_ba=calAdjointMatFromT(Tran_ab^(-1),"V");
     
 elseif(size(Tran_ab,1) == 3 && size(Tran_ab,2) == 3)   %��ת����
     T=composeTfromRandP(Tran_ab',zeros(3,1));   %�ȹ�����α任����
     X_ba=calAdjointMatFromT(T,"V");%ת�������任����
     
 elseif(size(Tran_ab,1) == 3 && size(Tran_ab,2) == 1)   %ƽ��
     T=composeTfromRandP(eye(3),-Tran_ab);   %�ȹ�����α任����
     X_ba=calAdjointMatFromT(T,"V");%ת�������任����
else
    error("û����������״�ı任����");
end

V_b=X_ba*V_a;

 

end

