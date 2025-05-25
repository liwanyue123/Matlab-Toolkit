function [V1] = calTwistFromOtherTwist(p0, V0, r_01, showType, rate, color)
% calTwistFromOtherTwist - 将一个 twist 从一个坐标系变换到另一个坐标系下表示，并可视化
%
% 输入参数：
%   p0        - V0 所在坐标系下 twist 的作用点位置（3x1 向量）
%   V0        - 在原始坐标系 A 中的 twist（6x1 向量）[w; v]
%   r_01      - 坐标系 B 相对于坐标系 A 的平移向量（3x1 向量）
%               表示从 A 到 B 的原点位移（即坐标变换 AB 的平移部分）
%   showType  - 显示类型参数（传递给 showTwist）
%   rate      - 显示缩放系数（用于放大向量以便可视化）
%   color     - 向量颜色（传递给 showTwist）
%
% 输出参数：
%   V1        - 将 V0 转换到 B 坐标系下表示的 twist 向量

T1 = calTranMatrixFromP(r_01);

Adj_T1 = calAdjointMatFromT(inv(T1), "V");

V1 = Adj_T1 * V0;

showTwist(p0 + r_01, V1, showType, rate, color);

end
