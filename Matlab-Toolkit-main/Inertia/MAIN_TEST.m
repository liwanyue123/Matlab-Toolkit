addpath(genpath("../TwistCalculation"));%旋量相关
addpath(genpath("../Coordinate"));%计算显示坐标系相关
addpath(genpath("../Matrix"));%矩阵相关
addpath(genpath("../Utils"));%矩阵相关
addpath(genpath("../simple"));%符号处理相关

addpath(genpath("../SphereTools"));%

clear

% 
% % 创建一个 SpatialInertia 对象
% mass = 2.78; % 质量
% com = [0.00914038 ;0.00519913 ;-0.11565513]; % 质心位置
%  % 转动惯量矩阵
% inertia = [0.01794633,-0.00012960,-0.00014935;
%            0.00012960,0.01908002,0.00037118;
%            0.00014935,-0.00037118,0.00244574];

% 创建一个 SpatialInertia 对象
mass = 2.8683; % 质量
com = [-0.00003971; 0.050385; -0.081143]; % 质心位置
% 转动惯量矩阵
inertia = [0.00633523, -0.00000646, -0.00000893;
           0.00000646, 0.00904244, -0.00037528;
           0.00000893, 0.00037528, 0.00593497];

res = checkInertia(inertia)

Inertia_server = SpatialInertia();
spatial_inertia= Inertia_server.SpatialInertiaFromMCI(mass, com, inertia);



pI=spatial_inertia.getPseudoInertia();
spatial_inertia2= Inertia_server.SpatialInertiaFromPseudoInertia(pI);




% 输出质心位置和转动惯量
fprintf('Center of Mass: [%f, %f, %f]\n', com(1), com(2), com(3));
fprintf('Inertia Tensor:\n');
disp(inertia);
fprintf('Center of Mass: [%f, %f, %f]\n', spatial_inertia.getCoM);
disp(spatial_inertia.getInertiaTensor());
fprintf('Center of Mass: [%f, %f, %f]\n', spatial_inertia2.getCoM);
disp(spatial_inertia2.getInertiaTensor());
% % 转换为伪惯性矩阵并输出
% pseudo_inertia = spatial_inertia.getPseudoInertia();
% fprintf('Pseudo Inertia Matrix:\n');
% disp(pseudo_inertia);
% 
% 绕 Y 轴翻转惯性矩阵并输出
flipped_inertia = spatial_inertia.flipAlongAxis('Y');
% fprintf('Flipped Inertia Matrix along Y Axis:\n');
% disp(flipped_inertia.inertia);


% 输出质心位置和转动惯量
fprintf('Center of Mass: [%f, %f, %f]\n', flipped_inertia.getCoM());
fprintf('Inertia Tensor:\n');
disp(flipped_inertia.getInertiaTensor());


















% syms theta1(t) t real
% 
% 
% theta=2*t;
% 
% % 1.rotation matrix
% R=calRotMatrix('Z',theta);
% 
% % 2.Derivative of a rotation matrix with respect to time (t)
% dR=calDotRotMatrix(R);
% 
% % 3.Lie algebra
% W= simplify(R'*dR);
% w=unHat(W);
% 
% % 4.Exp mapping
% exp_W=expm(W*t);
% 
% % % 5.Axis angle (Log mapping)
% % logm(R)
% % 
% % % 6.Axis angle (rodrigues)
% % rotMat2AxisAngle(R)
% 
% % --------------------------------------
% % Substitute specific values for analysis
% 
% %  绘制世界坐标系
% C_world=showWorldCoordinate(2);
% 
% run_time=1
% 
% % 1.rotation matrix
% R_data=eval(subs(R,t,run_time))
% 
% C_body1=genCoordinateCoord(C_world,expandMatTo4Dim(R_data));
% showCoodinate(C_body1,'C_{body}',1)
% 
% 
% % 2.Derivative of a rotation matrix with respect to time (t)
% dR_data=eval(subs(dR,t,run_time))
% 
% % 3.Lie algebra
% W_data=eval(subs(W,t,run_time))
% w_data=unHat(W_data)
% 
% % 4.Exp mapping
% exp_W_data=eval(subs(exp_W,t,run_time))
% 
% % % 5.Axis angle (Log mapping)
% temp=logm(R_data);
% vecAngle=unHat(temp)
% % 
% % % 6.Axis angle (rodrigues)
% [vec,angle]=rotMat2AxisAngle(R_data)
% 
% axis equal


 



