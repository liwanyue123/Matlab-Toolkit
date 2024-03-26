% 通过伪惯性矩阵初始化空间惯性
        function obj = SpatialInertiaFromPseudoInertia(P)
            h = unHat(P(1:3, 2:4));
            Ibar = trace(P(1:3, 1:3)) * eye(3) - P(1:3, 1:3);
            m = P(4, 4);
            obj.inertia(1:3, 1:3) = Ibar;
            obj.inertia(1:3, 4) = h;
            obj.inertia(4, 1:3) = h';
            obj.inertia(4, 4) = m ;
        end