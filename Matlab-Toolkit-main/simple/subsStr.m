function [K_sym_str] = subsStr(K_sym_begin,str_course,str_target)
% ��һ�����ź����е�ĳ�������滻����һ�����ţ���ʱ����һ���ַ�����
% ��Ϊ�ַ���ת����ʱ����֮ǰ�����еķ�����ȫ�ֱ���������֪����ô��
K_sym_str=char(K_sym_begin);

for k=1:max(size(str_course))
     K_sym_str=strrep(K_sym_str,str_course(k),str_target(k));
end

% K_sym_str=strrep(K_sym_str,str_course(1),str_target(1));
% K_sym_str=strrep(K_sym_str,str_course(2),str_target(2));
% K_sym_str=strrep(K_sym_str,str_course(3),str_target(3));
% % K_sym_end=simplify(str2sym(K_sym_str));
end

