function [text] = vec2Text(vec,num)
text='[';
for i=1:1:num
    text=[text num2str(vec(i))];
    if i==3
       text=[text,','];
    end
    if i~=num
        text=[text,' '];
    end
    
end
text=[text,']'];
end

