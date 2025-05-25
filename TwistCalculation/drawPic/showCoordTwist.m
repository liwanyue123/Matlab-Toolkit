function [] = showCoordTwist(Coord,Twist,show,rate)
%在某个坐标系Coord原点处显示，使用这个坐标系基底的一个旋量，注意是那个坐标系原点处，不然问题会变复杂
%show,因为同一个刚体上各点角速度一致，所以一般我们不想显示它
%rate是向量长度的缩放比例
%在世界坐标系的pot点处，显示旋量Twist

w=Twist(1:3)*rate;
v=Twist(4:6)*rate;
%我们要把这个坐标系的原点，还有旋量都转到世界坐标系上 


%用向量在坐标系的表示的转换来做，换个基底
pot=XYZ2p(Coord.p0)';%目标坐标系的原点在世界坐标系下的位置

%这是因为我坐标系定义的问题
X_w2coord=XYZ2p(Coord.x1)'-pot;
Y_w2coord=XYZ2p(Coord.y1)'-pot;
Z_w2coord=XYZ2p(Coord.z1)'-pot;

R_w2coord=[X_w2coord,Y_w2coord,Z_w2coord];
        
%把速度看出空间中一点，现在用世界坐标系来表示它
w=R_w2coord*w;
v=R_w2coord*v;

if(show=="all")
    circleArrow(pot ,w) ;
lineArrow(pot,v );
elseif(show=="w")
    circleArrow(pot ,w) ;

elseif(show=="v")
    lineArrow(pot,v ,"blue");
elseif(show=="null")
end
 
end

