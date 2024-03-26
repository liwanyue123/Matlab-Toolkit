        % 通过质量特性向量初始化空间惯性
        function obj = SpatialInertiaFromMassProperties(a)
            obj.inertia(1, 1) = a(5);
            obj.inertia(1, 2) = a(10);
            obj.inertia(1, 3) = a(9);
            obj.inertia(2, 1) = a(10);
            obj.inertia(2, 2) = a(6);
            obj.inertia(2, 3) = a(8);
            obj.inertia(3, 1) = a(9);
            obj.inertia(3, 2) = a(8);
            obj.inertia(3, 3) = a(7);
            cSkew = hat([a(2); a(3); a(4)]);
            obj.inertia(1:3, 4:6) = cSkew;
            obj.inertia(4:6, 1:3) = cSkew';
            obj.inertia(4:6, 4:6) = a(1) * eye(3);
        end