function [R] = exp_W(W,num)
%ָ��ӳ�䣬�������ͨ��̩��չ��ӳ�䵽��Ⱥ��
if ~(isnumeric(num) && isscalar(num) && num > 0 && num == floor(num))
    error('�������Ϊ������');
end
[n,m]=size(W);
if m ~= n
    error('���������һ������');
end
R=eye(n);
%̩��չ��
for i=1:1:num
    R=R+W^(i)/factorial(i)
end
end

