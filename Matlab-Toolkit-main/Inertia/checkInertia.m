function res = checkInertia(I)
    % 检查矩阵
    test_I = -I.';
    test_I(1:size(I, 1) + 1:end) = diag(I);
    res = isequal(I, test_I);
    if ~res
        error('The matrix is not antisymmetric.');
    end
end