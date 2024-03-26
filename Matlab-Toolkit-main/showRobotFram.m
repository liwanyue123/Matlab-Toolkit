function [T_foot] = showRobotFram(q,tau,p)


% 一次性赋值
[q1, q2, q3, q4, q5] = deal(q(1), q(2), q(3), q(4), q(5));
[t1, t2, t3, t4, t5] = deal(tau(1), tau(2), tau(3), tau(4), tau(5));
[l1, l2, l3, l4, l5, l6] = deal(p.l1, p.l2, p.l3, p.l4, p.l5, p.l6);

R1 = calTranMatrixFromAxisTheta('X',q1);
R2 = calTranMatrixFromAxisTheta('Z',q2);
R3 = calTranMatrixFromAxisTheta('Y',q3);
R4 = calTranMatrixFromAxisTheta('Y',q4);
R5 = calTranMatrixFromAxisTheta('Y',q5);

T1=R1;
T2=calTranMatrixFromP([0;0;-l1])*R2;
% T3=calTranMatrixFromP([0;0;-l2])*R3;
T3=R3;
T4=calTranMatrixFromP([0;0;-l3])*R4;
T5=calTranMatrixFromP([0;0;-l4])*R5;
T6=calTranMatrixFromP([0;0;-l5]);

C_world=showWorldCoordinate(0.2);

% hip_x
C_hip_x=genCoordinateCoord(C_world,T1);
showCoodinate(C_hip_x,'C_{hip_x}',0.1)
axis_hip_x=getRotorAxisFromR('X',T1(1:3,1:3))
F1=[axis_hip_x*t1;0;0;0;]
showScrew(getPosFromTranMatrix(T1),F1,"all",p.rate,'b',p.max_visual_tau,"F1")

% hip_z
C_hip_z=genCoordinateCoord(C_world,T1*T2);
showCoodinate(C_hip_z,'C_{hip_z}',0.1)
draw2dotLine(getCoordOriginPos(C_hip_x),getCoordOriginPos(C_hip_z))

% hip_y
C_hip_y=genCoordinateCoord(C_world,T1*T2*T3);
showCoodinate(C_hip_y,'C_{hip_y}',0.1)

% knee
C_knee=genCoordinateCoord(C_world,T1*T2*T3*T4);
showCoodinate(C_knee,'C_{knee}',0.1)
draw2dotLine(getCoordOriginPos(C_hip_y),getCoordOriginPos(C_knee))

% ankle
C_ankle=genCoordinateCoord(C_world,T1*T2*T3*T4*T5);
showCoodinate(C_ankle,'C_{ankle}',0.1)
draw2dotLine(getCoordOriginPos(C_knee),getCoordOriginPos(C_ankle))

C_foot=genCoordinateCoord(C_world,T1*T2*T3*T4*T5*T6);
showCoodinate(C_foot,'C_{foot}',0.07,'black')

T_foot=T1*T2*T3*T4*T5*T6;


end



 


 







