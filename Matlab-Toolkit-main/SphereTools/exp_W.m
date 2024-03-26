function [R] = exp_W(W,num)
%指数映射，将李代数通过泰勒展开映射到李群上
if ~(isnumeric(num) && isscalar(num) && num > 0 && num == floor(num))
    error('输入必须为正整数');
end
[n,m]=size(W);
if m ~= n
    error('输入必须是一个方阵');
end
R=eye(n);
%泰勒展开
for i=1:1:num
    R=R+W^(i)/factorial(i)
end
end

