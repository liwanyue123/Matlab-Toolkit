function [mat_des] = bottomLeftCorner(mat_res,block_size)
% ��ȡԭʼ����ĳߴ�
[Mat_row,Mat_col]=size(mat_res);
block_row=block_size(1);
block_col=block_size(2);
begin_row=Mat_row-block_row+1;

 mat_des=mat_res(begin_row:Mat_row,1:block_col);
% begin_col=Mat_col-block_col+1;
%  mat_des=mat_res(begin_row:Mat_row,begin_col:Mat_col);
end

