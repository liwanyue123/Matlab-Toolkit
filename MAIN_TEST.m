addpath(genpath("TwistCalculation/twistAndScrew"));%旋量相关
addpath(genpath("TwistCalculation/drawPic"));%旋量相关
addpath(genpath("Coordinate"));%计算显示坐标系相关
addpath(genpath("Matrix"));%矩阵相关
addpath(genpath("Utils"));%矩阵相关
addpath(genpath("simple"));%符号处理相关

addpath(genpath("SphereTools"));%

clear

angle1=45;
angle2=45;
l1=2;
l2=2;
l3=2;
 
R1=calTranMatrixFromR('Y',deg2rad(angle1));
R2=calTranMatrixFromR('Y',deg2rad(angle2));
T_w1=calTranMatrixFromP([l1,0,0])*R1;
T_12=calTranMatrixFromP([l2,0,0])*R2;
T_23=calTranMatrixFromP([l3,0,0]);
T_w2=T_w1*T_12;
T_w3=T_w1*T_12*T_23;

%画坐标系
C_world=showWorldCoordinate( 1 );%  绘制世界坐标系
C_body1=genCoordinateCoord(C_world,T_w1);
showCoodinate(C_body1,'C_{body1}',1)
C_body2=genCoordinateCoord(C_world,T_w2);
showCoodinate(C_body2,'C_{body2}',1)
C_body3=genCoordinateCoord(C_world,T_w3);
showCoodinate(C_body3,'C_{body3}',1)

% 构造坐標系首尾相連线段
lineList = [C_world.p0, C_body1.p0, C_body2.p0, C_body3.p0];
showLine(lineList, '-', 'b');


X_23=calAdjointMatFromT(T_23,'V');
X_13=calAdjointMatFromT(T_12*T_23,'V');
% 可以用转置和负组合，而不用取逆
X_32=inv(X_23) ;
X_31=inv(X_13) ;

% 计算雅可比矩阵 
S2=generateNormTwsit('Y', 'Rotational');
S1=generateNormTwsit('Y', 'Rotational');
% 得到末端点处的关节旋量场的值
S_32=X_32*S2;
S_31=X_31*S1;
% 雅可比每一列就是他们
J2=S_32;
J1=S_31;
J=[J1,J2];

% world系
F_w=generateNormTwsit('Z', 'Prismatic')*10;
% 转成末端坐标系   
T_3w=inv(T_w3) ;
X_3w=calAdjointMatFromT(T_3w,'V');
F_f=X_3w*F_w

tau=transpose(J)*F_f % 转的也是关节局部坐标系

view(3) 
axis equal

