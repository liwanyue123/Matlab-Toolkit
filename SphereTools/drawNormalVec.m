function [] = drawNormalVec(R,constant)
% �����ʼ����������ת���� R �������
v_x = R(1:3,1);
v_z = R(1:3,3);

% ������ת�������
quiver3(v_z(1), v_z(2), v_z(3), v_z(1)*constant, v_z(2)*constant, v_z(3)*constant, 'LineWidth', 2, 'Color', 'r')





end

