function [outCell] = strReplace(srcCell,desCell,beginStr,endStr)
outPointer = 0;
srcCellSize = size(srcCell);
desCellSize = size(desCell);

endRowNum = strmatch(beginStr,srcCell);%��ʼ�滻����λ��
range = 1 : endRowNum;
for i = range
    outPointer = outPointer + 1;
    outCell{outPointer} = srcCell{i};    %�ȳ�����srcCell��Ҫ�滻����ǰ����
end

range = 1 : desCellSize;
for i = range
    outPointer = outPointer + 1;
    outCell{outPointer} = desCell{i};   %����дҪ�滻����
end

beginRowNum = strmatch(endStr,srcCell); %�滻���ݽ���λ��
range = beginRowNum : srcCellSize;
for i = range
    outPointer = outPointer + 1;
    outCell{outPointer} = srcCell{i};   %���ų�����ʣ�����
end
outCell = outCell'
 