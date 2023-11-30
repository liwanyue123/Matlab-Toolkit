function [] = showLeeAlgorithm_so3(R,W)
w=  unHat(W);%从李代数提取旋转向量
% theta=norm(w);%旋转角度
% u=w/theta;%旋转单位向量
%将李代数的向量画在旋转矩阵x轴末端
X_axis=R(1:3,1);

quiver3(X_axis(1),X_axis(2),X_axis(3), w(1), w(2), w(3), 'LineWidth', 2, 'color', [1, 0.5, 0])


 

end

