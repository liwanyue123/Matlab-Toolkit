import torch
import time

 

class LegIKController:
    def __init__(self, robot_name="g1",device='cpu'):
        """
        初始化腿部逆运动学控制器
        
        参数:
            legID (int): 腿编号 0-左腿 1-右腿
            robot_name (str): 机器人型号
            device (str): 计算设备 ('cpu' or 'cuda')
        """

        self.device = device
     
        # 初始化固定参数
        self.leg_struct_left = self.get_leg_config_torch('left')
        self.leg_struct_right = self.get_leg_config_torch('right')
        self.joint_limits_left = self.get_joint_limits('left')
        self.joint_limits_right = self.get_joint_limits('right')
        self.initial_guess_left = self.get_initial_guess('left')
        self.initial_guess_right = self.get_initial_guess('right')

    # ===== 批量兼容的PyTorch工具函数 =====
    def genCrossMatrix(self,P):
        """批量生成叉乘矩阵 (batch_size, 3) -> (batch_size, 3, 3)"""
        zero = torch.zeros_like(P[:, 0])
        return torch.stack([
            torch.stack([zero, -P[:, 2], P[:, 1]], dim=1),
            torch.stack([P[:, 2], zero, -P[:, 0]], dim=1),
            torch.stack([-P[:, 1], P[:, 0], zero], dim=1)
        ], dim=1)

    def calAdjointMatFromT(self,T, str_VorF):
        """计算伴随矩阵 (batch_size, 4, 4) -> (batch_size, 6, 6)"""
        R = T[:, :3, :3]
        P = T[:, :3, 3]
        P_x = self.genCrossMatrix(P)

        if str_VorF == 'V':
            top = torch.cat([R, torch.zeros_like(R)], dim=2)
            bottom = torch.cat([torch.matmul(P_x, R), R], dim=2)
            return torch.cat([top, bottom], dim=1)
        elif str_VorF == 'F':
            top = torch.cat([R, torch.matmul(P_x, R)], dim=2)
            bottom = torch.cat([torch.zeros_like(R), R], dim=2)
            return torch.cat([top, bottom], dim=1)
        else:
            raise ValueError("str_VorF must be 'V' or 'F'")

    def generateNormTwist(self,axis, mode='Rotational'):
        """生成标准旋量 -> (6,)"""
        axes = {'X': [1, 0, 0], 'Y': [0, 1, 0], 'Z': [0, 0, 1]}
        if axis.upper() not in axes:
            raise ValueError("Axis must be 'X', 'Y' or 'Z'")
        vec = torch.tensor(axes[axis.upper()], device=device, dtype=torch.float32)
        if mode == 'Rotational':
            return torch.cat([vec, torch.zeros(3, device=device)])
        else:
            return torch.cat([torch.zeros(3, device=device), vec])

    def get_rotation_matrix(self,axis, theta):
        """批量生成旋转矩阵 (batch_size,) -> (batch_size, 4, 4)"""
        c = torch.cos(theta)
        s = torch.sin(theta)
        ones = torch.ones_like(c)
        zeros = torch.zeros_like(c)

        if axis == 'X':
            R = torch.stack([
                torch.stack([ones, zeros, zeros, zeros], dim=1),
                torch.stack([zeros, c, -s, zeros], dim=1),
                torch.stack([zeros, s, c, zeros], dim=1),
                torch.stack([zeros, zeros, zeros, ones], dim=1)
            ], dim=1)
        elif axis == 'Y':
            R = torch.stack([
                torch.stack([c, zeros, s, zeros], dim=1),
                torch.stack([zeros, ones, zeros, zeros], dim=1),
                torch.stack([-s, zeros, c, zeros], dim=1),
                torch.stack([zeros, zeros, zeros, ones], dim=1)
            ], dim=1)
        elif axis == 'Z':
            R = torch.stack([
                torch.stack([c, -s, zeros, zeros], dim=1),
                torch.stack([s, c, zeros, zeros], dim=1),
                torch.stack([zeros, zeros, ones, zeros], dim=1),
                torch.stack([zeros, zeros, zeros, ones], dim=1)
            ], dim=1)
        else:
            raise ValueError("Axis must be 'X', 'Y' or 'Z'")
        return R


    # ========== 腿部配置（预计算为张量） ==========
    def get_leg_config_torch(self, side='left'):
        side = side.lower()
        sign = 1 if side == 'left' else -1

        # 转换为PyTorch张量
        def make_tensor(arr):
            return torch.tensor(arr, device=self.device, dtype=torch.float32).unsqueeze(0)

        leg = [
            {'type': 'Y', 'name': 'hip_pitch', 'T': make_tensor(torch.eye(4))},
            {'type': 'X', 'name': 'hip_roll', 'T': make_tensor([
                [0.984744, 0, -0.174010, 0],
                [0, 1, 0, 0.052000 * sign],
                [0.174010, 0, 0.984744, -0.030465],
                [0, 0, 0, 1],
            ])},
            {'type': 'Z', 'name': 'hip_yaw', 'T': make_tensor([
                [1, 0, 0, 0.025001],
                [0, 1, 0, 0],
                [0, 0, 1, -0.124120],
                [0, 0, 0, 1],
            ])},
            {'type': 'Y', 'name': 'knee', 'T': make_tensor([
                [0.984744, 0, 0.174010, -0.078273],
                [0, 1, 0, 0.002149 * sign],
                [-0.174010, 0, 0.984744, -0.177340],
                [0, 0, 0, 1],
            ])},
            {'type': 'Y', 'name': 'ankle_pitch', 'T': make_tensor([
                [1, 0, 0, 0],
                [0, 1, 0, -0.000094 * sign],
                [0, 0, 1, -0.300010],
                [0, 0, 0, 1],
            ])},
            {'type': 'X', 'name': 'ankle_roll', 'T': make_tensor([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.017558],
                [0, 0, 0, 1],
            ])},
        ]
        # 预计算旋量
        for joint in leg:
            joint['twist'] = self.generateNormTwist(joint['type'])
        return leg
        
    def get_joint_limits(self, side='left'):
        """返回指定边（左/右）的关节角限制"""
        side = side.lower()
        if side == 'left':
            return torch.tensor([
                [-2.5307,  2.8798],  # hip_pitch
                [-0.5236,  2.9671],  # hip_roll
                [-2.7576,  2.7576],  # hip_yaw
                [-0.0873,  2.8798],  # knee
                [-0.8727,  0.5236],  # ankle_pitch
                [-0.2618,  0.2618],  # ankle_roll
            ], device=self.device)
        elif side == 'right':
            return torch.tensor([
                [-2.5307,  2.8798],  # hip_pitch
                [-2.9671,  0.5236],  # hip_roll
                [-2.7576,  2.7576],  # hip_yaw
                [-0.0873,  2.8798],  # knee
                [-0.8727,  0.5236],  # ankle_pitch
                [-0.2618,  0.2618],  # ankle_roll
            ], device=self.device)
        else:
            raise ValueError("Side must be 'left' or 'right'")

    def get_initial_guess(self,leg_side):
        assert leg_side in ['left', 'right']
        if leg_side == 'left':
            initial = torch.tensor([-0.2, -0.1, 0.02, 0.6, 0.05, 0.0], dtype=torch.float32, device=self.device)
        else:
            initial = torch.tensor([-0.2, 0.1, -0.02, 0.6, 0.05, 0.0], dtype=torch.float32, device=self.device)
        return initial

    # ========== 批量正运动学 ==========
    def forward_kinematics_batch(self, leg_struct, theta_list):
        """
        批量正运动学（设备安全版本）
        参数:
            leg_struct: 腿部结构（已确保在正确设备上）
            theta_list: (batch_size, 6) 输入关节角
            device: 可选，默认使用theta_list的设备
        返回: 
            x (batch_size, 3), T_cum (batch_size, 4, 4), J_foot (batch_size, 6, 6)
        """

        batch_size = theta_list.shape[0]
        
        # 确保所有临时张量都在正确设备上
        T_cum = torch.eye(4, device=self.device).unsqueeze(0).repeat(batch_size, 1, 1)
        J_base_cols = []

        for i, joint in enumerate(leg_struct):
            # 确保关节参数在正确设备上
            T_fixed = joint['T'].to(device).repeat(batch_size, 1, 1)
            twist = joint['twist'].to(device)
            
            # 计算关节变换（自动继承设备）
            theta_i = theta_list[:, i].to(device)
            T_joint = self.get_rotation_matrix(joint['type'], theta_i)
            
            # 更新累积变换
            T_cum = torch.matmul(T_cum, torch.matmul(T_fixed, T_joint))
            
            # 计算雅可比列
            adj_T = self.calAdjointMatFromT(T_cum.to(device), 'V')  # 双重确保
            J_i = torch.matmul(adj_T, twist.repeat(batch_size, 1).unsqueeze(-1)).squeeze(-1)
            J_base_cols.append(J_i)

        # 组合雅可比矩阵
        J_base = torch.stack(J_base_cols, dim=2).to(device)
        
        # 计算足端雅可比
        T_inv = torch.inverse(T_cum).to(device)  # 确保逆运算在正确设备
        Ad_inv = self.calAdjointMatFromT(T_inv, 'V')
        J_foot = torch.matmul(Ad_inv.to(device), J_base.to(device))
        
        return T_cum[:, :3, 3], T_cum, J_foot

    # ========== 批量数值逆运动学 ==========
    def numericIK_batch(self, leg_struct, limit, initial_guess, x_desired, max_iter=30, tol=1e-4):
        """
        添加了自适应功能的批量数值逆运动学
        包括：
        - 自适应阻尼系数 (lambda)
        - 自适应学习率
        - 关节限制
        - 最佳解记录
        - 定期重置到最佳解
        - 无改进时提前终止
        """
        batch_size = x_desired.shape[0]
        
        # 初始化关节角度 - 使用初始猜想
        # theta = initial_guess.clone().repeat(batch_size, 1)
        theta = initial_guess

        # 自适应参数
        lambda_val = torch.full((batch_size,), 0.1, device=self.device)  # 阻尼系数
        min_lambda = 0.01
        max_lambda = 1.0
        learning_rate = torch.ones(batch_size, device=self.device)*2  # 学习率
        min_learning_rate = 0.3
        reduction_factor = 0.8
        
        # 记录最佳解
        best_theta = theta.clone()
        best_error = torch.full((batch_size,), float('inf'), device=self.device)
        best_iter = torch.zeros(batch_size, dtype=torch.int, device=self.device)
        no_improvement_cnt = torch.zeros(batch_size, dtype=torch.int, device=self.device)
        
        # 收敛状态和迭代计数
        success = torch.zeros(batch_size, dtype=torch.bool, device=self.device)
        iters = torch.zeros(batch_size, dtype=torch.int, device=self.device)
        
        # 关节限制
        joint_lower = limit[:, 0]
        joint_upper = limit[:, 1]
        
        # 计算初始误差
        x_current, _, _ = self.forward_kinematics_batch(leg_struct, theta)
        prev_error = torch.norm(x_desired - x_current, dim=1)
        print(f'[IK] Initial error: {prev_error.mean().item():.5f} | Using initial guess')
        
        # 初始化最佳解
        best_theta.copy_(theta)
        best_error.copy_(prev_error)
        
        # 主迭代循环
        for iter_idx in range(max_iter):
            active_mask = ~success
            
            # 如果没有活跃样本，提前退出
            if not active_mask.any():
                break
                
            # 更新活跃样本的迭代计数器
            iters[active_mask] += 1
            
            # 计算当前位置和雅可比矩阵
            x_current, _, J_foot = self.forward_kinematics_batch(leg_struct, theta)
            
            # 计算当前误差
            error = x_desired - x_current
            err_norm = torch.norm(error, dim=1)
            
            # 更新最佳解
            improved = err_norm < best_error
            best_error[improved] = err_norm[improved]
            best_theta[improved] = theta[improved]
            best_iter[improved] = iter_idx
            no_improvement_cnt[~improved] += 1
            no_improvement_cnt[improved] = 0
            
            # 检查收敛
            converged = err_norm < tol
            success[converged] = True
            
            # 如果所有样本已收敛，提前退出
            if torch.all(success):
                print(f'[IK] Converged at iteration {iter_idx}')
                break
                
            # 提取雅可比矩阵的位置部分 (N, 3, 6)
            J = J_foot[:, 3:6, :]
            
            # 自适应调整参数
            error_ratio = torch.where(prev_error > 0, err_norm / prev_error, torch.ones_like(err_norm))
            
            # 根据误差变化调整阻尼和学习率
            mask_difficult = active_mask & (error_ratio > 0.95)
            lambda_val[mask_difficult] = torch.min(lambda_val[mask_difficult] * 1.2, torch.tensor(max_lambda, device=self.device))
            learning_rate[mask_difficult] = torch.max(learning_rate[mask_difficult] * reduction_factor, torch.tensor(min_learning_rate, device=self.device))
            
            mask_good = active_mask & (error_ratio < 0.7)
            lambda_val[mask_good] = torch.max(lambda_val[mask_good] * 0.8, torch.tensor(min_lambda, device=self.device))
            learning_rate[mask_good] = torch.min(learning_rate[mask_good] * 1.1, torch.tensor(1.0, device=self.device))
            
            # 为每个样本计算delta_theta
            delta_theta_list = []
            for i in range(batch_size):
                if not active_mask[i]:
                    delta_theta_list.append(torch.zeros(6, device=self.device))
                    continue
                    
                J_i = J[i]  # (3,6)
                err_i = error[i]  # (3,)
                
                # 阻尼最小二乘
                JtJ = J_i.t() @ J_i
                # 加入阻尼项
                damped = JtJ + lambda_val[i] * torch.eye(6, device=self.device)
                
                # 计算delta_theta
                delta_theta = torch.linalg.solve(damped, J_i.t() @ err_i)
                delta_theta *= learning_rate[i]
                
                # 自适应步长控制
                max_step = min(0.3, 0.1 + 0.1 * err_norm[i].item())
                step_norm = torch.norm(delta_theta).item()
                
                if step_norm > max_step:
                    delta_theta = delta_theta / step_norm * max_step
                    # print(f'[IK] Step clipped to {max_step:.4f} rad')
                
                delta_theta_list.append(delta_theta)
                
            delta_theta_mat = torch.stack(delta_theta_list)
            
            # 更新关节角度
            theta[active_mask] += delta_theta_mat[active_mask]
            
            # 应用关节限制
            theta = torch.min(torch.max(theta, joint_lower), joint_upper)
            
            # 定期重置到最佳解（每10次迭代）
            reset_mask = active_mask & (iter_idx % 10 == 0) & (iter_idx > best_iter + 2)
            if reset_mask.any():
                # print(f'[IK] Resetting {reset_mask.sum()} samples to best solution')
                theta[reset_mask] = best_theta[reset_mask]
                
            # 检查无改进的样本
            no_improve_mask = active_mask & (no_improvement_cnt > 20)
            if no_improve_mask.any():
                # print(f'[IK] Stopping {no_improve_mask.sum()} samples without improvement')
                theta[no_improve_mask] = best_theta[no_improve_mask]
                success[no_improve_mask] = best_error[no_improve_mask] < tol * 2
                
            # 保存当前误差供下一轮使用
            prev_error.copy_(err_norm)
        
        # 将未收敛但误差较小的样本标记为部分成功
        partial_success = ~success & (best_error < tol * 2)
        success[partial_success] = True
        theta[partial_success] = best_theta[partial_success]
        
        print(f'[IK] Completed: Success={success.sum().item()}/{batch_size}, Avg error={best_error.mean().item():.5f}')
        
        return theta, success, iters

    def ComputeIK(self, legID, positions, initial_guess, robot_name="g1"):
        """
        批量数值逆运动学求解 (支持多环境并行计算)

        参数:
            legID (int): 腿编号 0-左腿 1-右腿
            positions (Tensor): 目标位置 [batch_size, 3] (x,y,z)
            device (str): 计算设备 ('cpu' or 'cuda')

        返回:
            Tensor: 关节角度 [batch_size, 6] (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
            Tensor: 收敛状态 [batch_size]
            Tensor: 迭代次数 [batch_size]
        """
        
        # 获取腿部配置结构
        if legID == 0:
        # 运行数值IK求解
            theta_sol, success, iters = self.numericIK_batch(self.leg_struct_left,  self.joint_limits_left, initial_guess, positions.to(device))
        else:
            theta_sol, success, iters = self.numericIK_batch(self.leg_struct_right, self.joint_limits_right, initial_guess, positions.to(device))
    
        # 返回结果
        return theta_sol, success, iters

 

def benchmark_ik(device='cuda', num_envs=4096, num_tests=5):
    # 初始化控制器
    controller = LegIKController(device=device)
    
    # 生成随机目标位置 (num_envs, 3)
    torch.manual_seed(42)
    # positions = torch.rand((num_envs, 3), device=device) * torch.tensor([0.4, 0.2, 0.6], device=device) + \
    #             torch.tensor([0,0, -0.5], device=device)
    positions = torch.ones((num_envs, 3), device=device) * torch.tensor([0.1, 0.0, -0.5], device=device) + torch.rand((num_envs, 3), device=device)*0.1

    # 0
    initial_guess = [torch.zeros((num_envs, 6), device=device) for _ in range(2)]
    initial_guess[0] = controller.initial_guess_left.clone().repeat(num_envs, 1)
    initial_guess[1] = controller.initial_guess_right.clone().repeat(num_envs, 1)
    # # 预热GPU (运行10次小批量)
    # print("Warming up GPU...")
    # for _ in range(10):
    #     _ = controller.ComputeIK(0, positions[:100],initial_guess,device)
    
    # 正式测试
    print(f"\nBenchmarking {num_envs} environments for {num_tests} iterations...")
    start_time = time.time()
    for _ in range(num_tests):
        theta_sol, success, iters = controller.ComputeIK(0, positions, initial_guess[0], device)
        initial_guess[0]= theta_sol
        theta_sol, success, iters = controller.ComputeIK(1, positions, initial_guess[1], device)
        initial_guess[1]= theta_sol
    
    # 确保所有计算完成
    # torch.cuda.synchronize()
    total_time = time.time() - start_time
    
    # 计算统计数据
    avg_time = total_time / num_tests
    throughput = num_envs / avg_time
    
    # 验证计算精度
    x_ver, _, _ = controller.forward_kinematics_batch(controller.leg_struct_left, theta_sol)
    errors = torch.norm(x_ver - positions, dim=1)
    
    print("\n[Benchmark Results]")
    print(f"Device: {device.upper()}")
    print(f"Environments: {num_envs:,}")
    print(f"Iterations: {num_tests:,}")
    print(f"Total time: {total_time:.3f} seconds")
    print(f"Average time per batch: {avg_time*1000:.2f} ms")
    print(f"Throughput: {throughput:,.0f} envs/second")
    print(f"Success rate: {success.float().mean().item()*100:.1f}%")
    print(f"Average iterations: {iters.float().mean().item():.1f}")
    print(f"Average position error: {errors.mean().item():.6f} m")
    
    # 示例输出前3个结果
    print("\nSample results (first 3 environments):")
    for i in range(3):
        print(f"\nEnv {i}: Target={positions[i].tolist()}")
        print(f"Joints: {theta_sol[i].tolist()}")
        print(f"Success: {success[i].item()} | Iters: {iters[i].item()}")
        print(f"Error: {errors[i].item():.6f} m")

if __name__ == "__main__":
    # 自动选择设备
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device.upper()}")
    
    # 运行性能测试
    benchmark_ik(device=device)
 
 
 