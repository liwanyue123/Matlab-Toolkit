function [coef] = collectCoef(func,Theta)
%��ȡϵ��
 Theta=subTime(Theta)
coef=equationsToMatrix(func, Theta)
end

