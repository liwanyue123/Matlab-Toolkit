    % 通过给定的6x6惯性矩阵初始化空间惯性
        function obj = SpatialInertiaFromMatrix(inertia)
            obj.inertia = inertia;
        end