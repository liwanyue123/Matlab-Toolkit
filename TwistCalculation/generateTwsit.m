function [S1] = generateTwsit(r1,s1,h,isShow)
 if isShow==true
showVector([0,0,0],r1,'g')
showVector(r1,s1 ,'r')
 end
s10=cross(r1,s1);
S1=[s1,s10+h*s1]';
end

