function [] = drawTangentVec(R,scale)
% �����ʼ����������ת���� R �������
v_x = R(1:3,1);
v_z = R(1:3,3);

% ������ת�������
quiver3(v_z(1), v_z(2), v_z(3), v_x(1)*scale, v_x(2)*scale, v_x(3)*scale, 'LineWidth', 2, 'Color', 'b')





end

