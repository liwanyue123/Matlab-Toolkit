function [] = drawCircle()
% ------------绘制单位圆------------
r = 1;      % 设置圆的半径为1
% 生成一个圆周上的角度数组，从0度到360度，间隔为1度
theta_circle = linspace(0, 2*pi, 361);
% 计算圆上的x坐标和y坐标
x = r*cos(theta_circle);
y = r*sin(theta_circle);
% 绘制圆
plot(x, y);
end

