function [] = showLeeAlgorithm_so3(R,W)
w=  unHat(W);%���������ȡ��ת����
% theta=norm(w);%��ת�Ƕ�
% u=w/theta;%��ת��λ����
%�������������������ת����x��ĩ��
X_axis=R(1:3,1);

quiver3(X_axis(1),X_axis(2),X_axis(3), w(1), w(2), w(3), 'LineWidth', 2, 'color', [1, 0.5, 0])


 

end

