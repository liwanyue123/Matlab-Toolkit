function [] = drawSphere()
% ���ɵ�λ��ı���
[x,y,z] = sphere;
% ���Ƶ�λ��
surf(x, y, z, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1)
hold on

end

