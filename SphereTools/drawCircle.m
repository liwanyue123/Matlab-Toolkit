function [] = drawCircle()
% ------------���Ƶ�λԲ------------
r = 1;      % ����Բ�İ뾶Ϊ1
% ����һ��Բ���ϵĽǶ����飬��0�ȵ�360�ȣ����Ϊ1��
theta_circle = linspace(0, 2*pi, 361);
% ����Բ�ϵ�x�����y����
x = r*cos(theta_circle);
y = r*sin(theta_circle);
% ����Բ
plot(x, y);
end

