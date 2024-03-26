

% addpath(genpath("Utils"));%矩阵相关


function [p, J, Jd, Rf] = ComputeFootPositionAndJacobian(leg, q, qd,p)
% Constants

[l1, l2, l3, l4, l5, l6] = deal(p.l1, p.l2, p.l3, p.l4, p.l5, p.l6);
bodyLength=p.bodyLength;
bodyWidth=p.bodyWidth;


side_sign = 1;
if (leg == 0) || (leg == 2)
    side_sign = -1;
end

p_hip = zeros(3, 1);
if (leg == 0 || leg == 1)
    p_hip(1) = bodyLength;
else
    p_hip(1) = -bodyLength;
end
if (leg == 1 || leg == 3)
    p_hip(2) = bodyWidth;
else
    p_hip(2) = -bodyWidth;
end

% 一次性赋值
[q1, q2, q3, q4, q5] = deal(q(1), q(2), q(3), q(4), q(5));
[dq1, dq2, dq3, dq4, dq5] = deal(qd(1), qd(2), qd(3), qd(4), qd(5));

ax = [1; 0; 0];
ay = [0; 1; 0];
az = [0; 0; 1];

R1 = calRotMatrix('X',q1);
R2 = calRotMatrix('Z',q2);
R3 = calRotMatrix('Y',q3);
R4 = calRotMatrix('Y',q4);
R5 = calRotMatrix('Y',q4);

L5 = R1 * R2 * R3 * R4 * R5 * [l6; 0; -l5];
L4 = L5 + R1 * R2 * R3 * R4 * [0; 0; -l4];
L3 = L4 + R1 * R2 * R3 * [0; 0; -l3];
L2 = L3 + R1 * R2 * [0; 0; -l2];
L1 = L2 + R1 * [0; 0; -l1];

Jw = [R1 * ax, ...
    R1 * R2 * az,...
    R1 * R2 * R3 * ay,...
    R1 * R2 * R3 * R4 * ay,...
    R1 * R2 * R3 * R4 * R5 * ay]

Jl = [cross(Jw(:,1), L1),...
    cross(Jw(:,2), L2), ...
    cross(Jw(:,3), L3), ...
    cross(Jw(:,4), L4),...
    cross(Jw(:,5), L5)];

W0 = [0; 0; 0];
W1 = Jw(:,1) * dq1;
W2 = W1 + Jw(:,2) * dq2;
W3 = W2 + Jw(:,3) * dq3;
W4 = W3 + Jw(:,4) * dq4;
W5 = W4 + Jw(:,5) * dq5;

V5 = Jl(:,5) * dq5;
V4 = V5 + Jl(:,4) * dq4;
V3 = V4 + Jl(:,3) * dq3;
V2 = V3 + Jl(:,2) * dq2;
V1 = V2 + Jl(:,1) * dq1;

dJw = [cross(W1, Jw(:,1)),...
    cross(W2, Jw(:,2)), ...
    cross(W3, Jw(:,3)), ...
    cross(W4, Jw(:,4)),...
    cross(W5, Jw(:,5))];
dJl = [cross(V1 + cross(W0, L1), Jw(:,1)) + cross(L1, dJw(:,1)), ...
    cross(V2 + cross(W1, L2), Jw(:,2)) + cross(L2, dJw(:,2)), ...
    cross(V3 + cross(W2, L3), Jw(:,3)) + cross(L3, dJw(:,3)), ...
    cross(V4 + cross(W3, L4), Jw(:,4)) + cross(L4, dJw(:,4)), ...
    cross(V5 + cross(W4, L5), Jw(:,5)) + cross(L5, dJw(:,5))];

% Body frame
p = L1 + [p_hip(1); p_hip(2); 0];
J = [Jw;Jl];
Jd = [dJw;dJl];
Rf = R1 * R2 * R3 * R4 * R5;
end
