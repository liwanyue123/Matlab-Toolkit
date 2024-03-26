addpath(genpath("TwistCalculation"));%旋量相关
addpath(genpath("Coordinate"));%计算显示坐标系相关
addpath(genpath("Matrix"));%矩阵相关
addpath(genpath("Utils"));%矩阵相关
addpath(genpath("simple"));%符号处理相关

addpath(genpath("SphereTools"));%

clear


syms theta1(t) t real


theta=2*t;

% 1.rotation matrix
R=calRotMatrix('Z',theta);

% 2.Derivative of a rotation matrix with respect to time (t)
dR=calDotRotMatrix(R);

% 3.Lie algebra
W= simplify(R'*dR);
w=unHat(W);

% 4.Exp mapping
exp_W=expm(W*t);

% % 5.Axis angle (Log mapping)
% logm(R)
% 
% % 6.Axis angle (rodrigues)
% rotMat2AxisAngle(R)

% --------------------------------------
% Substitute specific values for analysis
 
%  绘制世界坐标系
C_world=showWorldCoordinate(2);

run_time=1

% 1.rotation matrix
R_data=eval(subs(R,t,run_time))

C_body1=genCoordinateCoord(C_world,expandMatTo4Dim(R_data));
showCoodinate(C_body1,'C_{body}',1)


% 2.Derivative of a rotation matrix with respect to time (t)
dR_data=eval(subs(dR,t,run_time))

% 3.Lie algebra
W_data=eval(subs(W,t,run_time))
w_data=unHat(W_data)

% 4.Exp mapping
exp_W_data=eval(subs(exp_W,t,run_time))

% % 5.Axis angle (Log mapping)
temp=logm(R_data);
vecAngle=unHat(temp)
% 
% % 6.Axis angle (rodrigues)
[vec,angle]=rotMat2AxisAngle(R_data)

% 设置相机的角度
azimuth = 45; % 方位角
elevation = 30; % 仰角
view(azimuth, elevation); % 设置相机角度

axis equal


 











% % %  绘制世界坐标系
% C_world=showWorldCoordinate( 1 );
% dt=0.1;
% omega=2;
% angle1=10;
% angle2=angle1+omega*dt;
% %只要限制只有z轴能选择，那么就是SO(2),我们依旧以三维的视角观察它
% T1=calRotMatrix('Z',deg2rad(angle1));
% T2=calRotMatrix('Z',deg2rad(angle2));
% %画坐标系
% C_body1=genCoordinateCoord(C_world,T1);
% showCoodinate(C_body1,'C_{body}',1)
% C_body2=genCoordinateCoord(C_world,T2);
% showCoodinate(C_body2,'C_{body}',1)

% R1=T1(1:3,1:3)
% R2=T2(1:3,1:3)
% 
% W1 = logm(R1)%对数映射出李代数
% w1=  unHat(W1)%从李代数提取旋转向量
% 
% W2 = logm(R2)%对数映射出李代数
% w2=  unHat(W2)%从李代数提取旋转向量


%分析轴角
% angle1=30;
% T1=calRotMatrix('Z',deg2rad(angle1));
% R1=T1(1:3,1:3)
% v=unHat(logm(R1))
% rad2deg(v(3))
% %计算角速度
% 
% rodrigues([0;0;1],pi/6)




% show_w(w)%画出旋转向量
% theta=norm(w)%旋转角度
% u=w/theta;%旋转单位向量