function [A,b] = equationsToMatrix_NonLinear(eq, x)
%EQUATIONSTOMATRIX equationsToMatrix ���ڷ����Է���
% �� eq �зֽ������ x������ eq = Ax + b 
% eq �� x �в���Ҫ�����Ե�
% eq ���������������飬���� x �����Ƿ�������

assert(isa(eq,'sym'), 'Equations must be symbolic')
assert(isa(x,'sym'), 'Vector x must be symbolic')

n = numel(eq);
m = numel(x);

A = repmat(sym(0),n,m);

for i = 1:n % ѭ������
    [c,p] = coeffs(eq(i),x); % �����̷���Ϊ x(1)...x(n) ��ϵ�����ݣ�
    for k = 1:numel(p) % ѭ��ͨ���ҵ�����/ϵ��
        for j = 1:m % ѭ��ͨ�� x(1).. .x(n)
            if has(p(k),x(j))
                % ���� c(k)*p(k) ת��Ϊ A���ֽ�� x(j)
                A(i,j) = A(i,j) + c(k)*p(k)/x(j);
                break% ������һ�� c(k+1), p(k+1) 
            end
        end
    end
end

b = simplify(eq - A*x,'ignoreanalyticconstraints',true); % ȷ����ȫȡ������

end