function [mat] = eOmega2Mat(w,theta)
%������e^([w]*theta).ʹ��̩��ָ��չ��
A=vectorToSkewMat(w);
I3=identityMat(3);
mat=I3;
i=1;
n=5;%̩��չ��5�����
while(i<=n)
part_i=(theta*A)^i/factorial(i);
mat=mat+part_i;
i=i+1;
end
end 
