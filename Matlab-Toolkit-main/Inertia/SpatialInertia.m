classdef SpatialInertia
    properties
        inertia % 6x6 空间惯性矩阵
    end
    
    methods
        % ========== initial ========== 
        
        % 通过质量、质心和惯性矩阵初始化空间惯性
        function obj = SpatialInertiaFromMCI(obj,mass, com, inertia)
            cSkew = hat(com); % 计算质心的反对称矩阵
            % 构造6x6空间惯性矩阵
            obj.inertia(1:3, 1:3) = inertia - mass * cSkew * cSkew;
            obj.inertia(1:3, 4:6) = mass * cSkew;
            obj.inertia(4:6, 1:3) = -mass * cSkew;
            obj.inertia(4:6, 4:6) = mass * eye(3);
        end
        
        % % 通过给定的6x6惯性矩阵初始化空间惯性
        function obj = SpatialInertiaFromMatrix(obj,inertia)
            obj.inertia = inertia;
        end

        % % 通过质量特性向量初始化空间惯性
        % function obj = SpatialInertiaFromMassProperties(a)
        %     obj.inertia(1, 1) = a(5);
        %     obj.inertia(1, 2) = a(10);
        %     obj.inertia(1, 3) = a(9);
        %     obj.inertia(2, 1) = a(10);
        %     obj.inertia(2, 2) = a(6);
        %     obj.inertia(2, 3) = a(8);
        %     obj.inertia(3, 1) = a(9);
        %     obj.inertia(3, 2) = a(8);
        %     obj.inertia(3, 3) = a(7);
        %     cSkew = hat([a(2); a(3); a(4)]);
        %     obj.inertia(1:3, 4:6) = cSkew;
        %     obj.inertia(4:6, 1:3) = cSkew';
        %     obj.inertia(4:6, 4:6) = a(1) * eye(3);
        % end
        % 
        % % 通过伪惯性矩阵初始化空间惯性
        function obj = SpatialInertiaFromPseudoInertia(obj,P)
            I = zeros(6);
            m = P(4, 4);
            h = P(1:3, 4);
            E = P(1:3, 1:3);
            Ibar = trace(E) * eye(3) - E;
            I(1:3, 1:3) = Ibar;
            I(1:3, 4:6) = hat(h);
            I(4:6, 1:3) = hat(h)';
            I(4:6, 4:6) = m * eye(3);
            obj.inertia = I;
        end
        
        % ========== func ========== 

        % 转换为质量特性向量
        function massProps = asMassPropertyVector(obj)
            m = obj.inertia(6, 6);
            h = unHat(obj.inertia(1:3, 4:6));
            Ixx = obj.inertia(1, 1);
            Iyy = obj.inertia(2, 2);
            Izz = obj.inertia(3, 3);
            Ixy = obj.inertia(2, 1);
            Ixz = obj.inertia(3, 1);
            Iyz = obj.inertia(3, 2);
            massProps = [m; h; Ixx; Iyy; Izz; Ixy; Ixz; Iyz];
        end

        function m = getMass(obj)
           m=obj.inertia(6,6);
        end


        function CoM = getCoM(obj)
           m=obj.getMass();
           CoM_skew =[eye(3)/m ,zeros(3,3)]*obj.inertia*[zeros(3,3);eye(3) ];
           CoM=unHat(CoM_skew);

        end

        % 获取转动惯量张量
        function inertiaTensor = getInertiaTensor(obj)
            m = obj.inertia(6, 6);
            mcSkew = obj.inertia(1:3, 4:6);
            I_rot = obj.inertia(1:3, 1:3) - mcSkew * mcSkew' / m;
            inertiaTensor = I_rot;
        end
        
        % 获取伪惯性矩阵
        function P = getPseudoInertia(obj)
            h = unHat(obj.inertia(1:3, 4:6));
            Ibar = obj.inertia(1:3, 1:3);
            m = obj.inertia(6, 6);
            P = zeros(4);
            P(1:3, 1:3) = 0.5*trace(Ibar) * eye(3) -Ibar;
            P(1:3, 4) = h;
            P(4, 1:3) = h';
            P(4, 4) = m;
        end
        
        % 绕指定轴翻转惯性矩阵
        function obj = flipAlongAxis(obj, axis)
            P = obj.getPseudoInertia();
            X = eye(4);
            if axis == 'X'
                X(1, 1) = -1;
            elseif axis == 'Y'
                X(2, 2) = -1;
            elseif axis == 'Z'
                X(3, 3) = -1;
            end
            P = X * P * X;
            obj = SpatialInertiaFromPseudoInertia(obj,P);
        end
    end
end
