function [mathematica_str] = matlab2Mathematica(matlab_str)
str=matlab_str

str=regexprep(str,'[','{');
str=regexprep(str,']','}');

%sin,cosԲ����ת�ɷ�����
str=regexprep(str,'sin\((.*?)\)','Sin[$1]');
str=regexprep(str,'cos\((.*?)\)','Cos[$1]');
%����theta_1_dd��� D[theta1[t],{t,2}]
str=regexprep(str,'theta_(.)_dd','D[theta$1[t],{t,2}]');
%����theta_1_d��� D[theta1[t],t]
str=regexprep(str,'theta_(.)_d','D[theta$1[t],t]');
%s������Ӧtheta_1���theta1
str=regexprep(str,'theta_(.)','theta$1');

mathematica_str=str;
end

