function [symfun_des] = addTime(symfun_res)
 
%����ʽ�����ӣ�t��
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 real
syms L1 L2 L3    a1 a2 a3 b1 real
syms g m1 m2 m3 real
syms theta_1(t) theta_2(t) theta_3(t) t real
syms theta_1_d(t) theta_2_d(t) theta_3_d(t)   real
syms theta_1_dd(t) theta_2_dd(t) theta_3_dd(t)   real
% symfun_text=char(symfun_res);%�ַ���
% symfun_text=WordReplace( symfun_text,'add') ;%�滻�У�t���ı���
% symfun_des=eval(cell2mat(symfun_text));%��ԭ�ɷ���
symfun_des=eval(symfun_res);%��ԭ�ɷ���
end

%  'matrix([[-L1*       theta_1_d                  *sin(theta_1)], [0], [0]])'
% {'matrix([[-L1*       diff(theta_1(t)(t), t)        *sin(theta_1(t))], [0], [0]])'}