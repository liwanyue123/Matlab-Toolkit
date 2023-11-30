function [func_res] = WordReplace( varargin)

%�����ַ������ӹ���֮������ַ���
func_src=varargin{1};
if (nargin==2)
commond=varargin{2};
elseif (nargin==1)
commond='sub';
end

%�����ַ������滻���еĴʻ�󴫳��ַ���
str_theta={'theta_1','theta_2','theta_3'};

if(commond=='sub')
    %-----------------------------���׵���------------------------------
            %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='diff(A(t), t, t)';
        word_symbol='A_dd';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end
    func_res = subsStr(func_src,str_course,str_target);
    
        %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))


        word_t='diff(A_d(t), t)';%diff(theta_1_d(t), t),A��theta_1
        word_symbol='A_dd';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end
    func_res = subsStr(func_src,str_course,str_target);
    
    
      %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='A_dd(t)';
        word_symbol='A_dd';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end
    func_res = subsStr(func_res,str_course,str_target);
    
    
        %-----------------------------һ�׵���------------------------------
    %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='diff(A(t), t)';
        word_symbol='A_d';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end
    func_res = subsStr(func_res,str_course,str_target);
    
     %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='A_d(t)';
        word_symbol='A_d';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end
    func_res = subsStr(func_res,str_course,str_target);
    
        %-----------------------------��ͨ����------------------------------
    %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='A(t)';
        word_symbol='A';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end 
    func_res = subsStr(func_res,str_course,str_target);
     
    
elseif(commond=='add')
 
     %-----------------------------���׵���------------------------------
   
    %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='A_dd';
        word_symbol='diff(A_d(t), t)';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end 
    func_res = subsStr(func_res,str_course,str_target);
             %-----------------------------һ�׵���------------------------------
            %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='A_d';
        word_symbol='diff(A(t), t)';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end
    func_res = subsStr(func_src,str_course,str_target);
    
     %-----------------------------��ͨ����------------------------------
     
         %����Ҫ�滻��Դ�ʻ��Ŀ��ʻ�
    for k=1:max(size(str_theta))
        word_t='A';
        word_symbol='A(t)';
        str_course(k)=strrep(word_t,'A',str_theta(k));
        str_target(k)=strrep(word_symbol,'A',str_theta(k));
    end 
    func_res = subsStr(func_res,str_course,str_target);
end


end

