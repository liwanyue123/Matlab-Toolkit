 
 
function [funcAfterDiff] = dif(varargin)

%������Ǹ��ַ�������
narginchk(1,2)
func= varargin{1};


if nargin==1%��ʱ����ȫ��
    
    %Ϊ���ܶ�t�󵼣������Ҫ��A(t),ƫ������A��A(t)����
    funcAfterDiff=diff(func);
elseif nargin==2%��ƫ����ֱ����ͺ���
    %����ʱ��
    variable= varargin{2};
    func=subTime(func) ;
    variable=subTime(variable) ;
    %ƫ��
    funcAfterDiff=diff(func,variable);
     %����ʱ��
%     funcAfterDiff=addTime(funcAfterDiff)
end
funcAfterDiff=simplify(funcAfterDiff);
end


