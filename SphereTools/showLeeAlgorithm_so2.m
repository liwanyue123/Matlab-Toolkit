function [] = showLeeAlgorithm_so2(W)
w=  unHat(W);%���������ȡ��ת����
theta=norm(w)%��ת�Ƕ�
u=w/theta;%��ת��λ����

direction=1;
if w(3)<0
   direction=-1;
end
start_point = [1, 0];  % �滻Ϊʵ�ʵ���ʼ������
end_point = [1, direction*theta];  % �滻Ϊʵ�ʵĽ���������

plot([start_point(1), end_point(1)], [start_point(2), end_point(2)], 'color', [1, 0.5, 0],'LineWidth', 2);

end

