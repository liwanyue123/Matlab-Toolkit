function [] = showLeeAlgorithm_so2(W)
w=  unHat(W);%从李代数提取旋转向量
theta=norm(w)%旋转角度
u=w/theta;%旋转单位向量

direction=1;
if w(3)<0
   direction=-1;
end
start_point = [1, 0];  % 替换为实际的起始点坐标
end_point = [1, direction*theta];  % 替换为实际的结束点坐标

plot([start_point(1), end_point(1)], [start_point(2), end_point(2)], 'color', [1, 0.5, 0],'LineWidth', 2);

end

