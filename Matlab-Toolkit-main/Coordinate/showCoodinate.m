function [] = showCoodinate(C,name,rate,varargin)
%C.p0֮��Ķ�����ά��C.p0��x����㣬C.x1��x����յ�
% quiver3Ҫ�������λ�ã���������������
%rate�Ǳ���������1����̫���ˣ����ÿ�
hold on

vec_X=[ C.x1.X-C.p0.X, C.x1.Y-C.p0.Y, C.x1.Z-C.p0.Z ];
vec_Y=[ C.y1.X-C.p0.X, C.y1.Y-C.p0.Y, C.y1.Z-C.p0.Z ];
vec_Z=[ C.z1.X-C.p0.X, C.z1.Y-C.p0.Y, C.z1.Z-C.p0.Z ];

color_list={'red','green','blue'};
%�������廻һ����ɫ
if nargin < 4
    % ���û�д�����ɫ������ʹ��Ĭ��ֵ
    text(C.p0.X,C.p0.Y,C.p0.Z,name,'color','r');
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,  vec_X(1),vec_X(2),vec_X(3) ,rate,'r')
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,  vec_Y(1),vec_Y(2),vec_Y(3) ,rate,'g')
    quiver3(C.p0.X,  C.p0.Y,  C.p0.Z,  vec_Z(1),vec_Z(2),vec_Z(3) ,rate,'b')

else
    color = varargin{1}; % ����ʹ�ô��ݵ���ɫ����
    text(C.p0.X,C.p0.Y,C.p0.Z,name,'color',color);
    quiver3(C.p0.X, C.p0.Y, C.p0.Z, C.x1.X-C.p0.X, C.x1.Y-C.p0.Y, C.x1.Z-C.p0.Z, rate, 'color', color)
    quiver3(C.p0.X, C.p0.Y, C.p0.Z, C.y1.X-C.p0.X, C.y1.Y-C.p0.Y, C.y1.Z-C.p0.Z, rate, 'color', color)
    quiver3(C.p0.X, C.p0.Y, C.p0.Z, C.z1.X-C.p0.X, C.z1.Y-C.p0.Y, C.z1.Z-C.p0.Z, rate, 'color', color)
    color_list={color,color,color};
end


% �����ǩλ��
vec_X=vec_X*rate;
vec_Y=vec_Y*rate;
vec_Z=vec_Z*rate;

% ����������ǩ
text(C.p0.X+vec_X(1),  C.p0.Y+vec_X(2), C.p0.Z+vec_X(3), 'x', 'color', color_list{1},  'FontSize', 14)
text(C.p0.X+vec_Y(1),  C.p0.Y+vec_Y(2), C.p0.Z+vec_Y(3), 'y', 'color', color_list{2},  'FontSize', 14)
text(C.p0.X+vec_Z(1),  C.p0.Y+vec_Z(2), C.p0.Z+vec_Z(3), 'z', 'color', color_list{3},  'FontSize', 14)


end

