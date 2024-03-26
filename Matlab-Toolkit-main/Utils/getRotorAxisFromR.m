function [v] = getRotorAxisFromR(axis,R)
select_vec=[0;0;0];
if axis == 'X'
    select_vec=[1;0;0];
elseif axis == 'Y'
    select_vec=[0;1;0];
elseif axis == 'Z'
    select_vec=[0;0;1];
end
v=R*select_vec;

end

