% =========================================================================
% 固定翼/导弹差分平坦度 (Differential Flatness) 反向梯度推导脚本
% 用于计算 J 关于 v, a, j 的梯度，采用符号计算并生成可用的 C++ 代码
% =========================================================================

clear; clc;

%% 1. 定义符号变量
% 状态变量：速度、加速度、加加速度
syms v0 v1 v2 real
syms a0 a1 a2 real
syms j0 j1 j2 real
syms grav real % 重力加速度 g

v = [v0; v1; v2];
a = [a0; a1; a2];
jer = [j0; j1; j2];

% 传入的误差/代价梯度（上游传递回来的梯度 dJ/du  )
syms dJ_dnt dJ_dnn dJ_dmu real
syms dJ_domg0 dJ_domg1 dJ_domg2 real % omg0, omg1, omg2 分别对应三个角速度分量的梯度
dJ_domg = [dJ_domg0; dJ_domg1; dJ_domg2];

%% 2. 正向平坦度映射过程 (Forward Map 的符号化重写)
% -- 基坐标系推导 --
% 计算气流坐标系x轴单位向量 i_a 在大地系中的投影
v_norm = sqrt(v.' * v);
i_a = v / v_norm;

% 计算气流坐标系y轴单位向量 j_a 在大地系中的投影 
a_g = a - [0; 0; grav]; % 过载加速度在大地系中的投影
v_cross_ag = cross(v, a_g);
v_cross_ag_norm = sqrt(v_cross_ag.' * v_cross_ag);
j_a = v_cross_ag / v_cross_ag_norm;

% 计算气流坐标系z轴单位向量 K_a 在大地系中的投影
k_a = cross(i_a, j_a);

% -- 控制量推导 --
n_t = dot(a_g, i_a) / grav; % 切向过载
n_n = -dot(a_g, k_a) / grav; % 法向过载

% mu = atan2(j_a(3), k_a(3))  
mu = atan2(j_a(3), k_a(3)); % 速度滚转角

% -- 角速度角加速度推导 --
v_dot_a = dot(v, a);
i_a_dot = (a * v_norm - v * v_dot_a / v_norm) / (v_norm^2);

cross_dot = cross(a, a_g) + cross(v, jer);
cross_dot_inner = dot(v_cross_ag, cross_dot);
j_a_dot = (cross_dot * v_cross_ag_norm - v_cross_ag * cross_dot_inner / v_cross_ag_norm) / (v_cross_ag_norm^2);

% omg = [omg_x; omg_y; omg_z]
omg0 = dot(k_a, j_a_dot);
omg1 = -dot(k_a, i_a_dot);
omg2 = dot(j_a, i_a_dot);

%% 3. 链式法则求总重导数 (Backward)
% 我们假设总代价泛函 J，已知 J 对输出量的偏导 
% 对 v 的总偏导：
fprintf('开始计算雅可比与链式推导...\n');
dJ_dv = dJ_dnt * jacobian(n_t, v) + ...
        dJ_dnn * jacobian(n_n, v) + ...
        dJ_dmu * jacobian(mu, v) + ...
        dJ_domg0 * jacobian(omg0, v) + ...
        dJ_domg1 * jacobian(omg1, v) + ...
        dJ_domg2 * jacobian(omg2, v);

% 对 a 的总偏导：
dJ_da = dJ_dnt * jacobian(n_t, a) + ...
        dJ_dnn * jacobian(n_n, a) + ...
        dJ_dmu * jacobian(mu, a) + ...
        dJ_domg0 * jacobian(omg0, a) + ...
        dJ_domg1 * jacobian(omg1, a) + ...
        dJ_domg2 * jacobian(omg2, a);

% 对 jer 的总偏导：
dJ_dj = dJ_dnt * jacobian(n_t, jer) + ...
        dJ_dnn * jacobian(n_n, jer) + ...
        dJ_dmu * jacobian(mu, jer) + ...
        dJ_domg0 * jacobian(omg0, jer) + ...
        dJ_domg1 * jacobian(omg1, jer) + ...
        dJ_domg2 * jacobian(omg2, jer);

%% 4. 提取为 matlab 函数
% MATLAB 的 ccode 函数可以直接将符号推导结果转化为 matlab 函数！
% 为了方便阅读和代码优化，防止生成的单行过长，使用中间子表达式简化：
fprintf('正在生成优化的 matlab 代码文件...\n');

dJ_dv = dJ_dv.'; % 转置为列向量，方便阅读
dJ_da = dJ_da.';
dJ_dj = dJ_dj.';

% 整理所有输入的变量，明确自变量顺位
vars_list = {v, a, jer, grav, dJ_dnt, dJ_dnn, dJ_dmu, dJ_domg0, dJ_domg1, dJ_domg2};

% 使用 matlabFunction 直接生成带有中间变量（t1, t2...）的 C++ 文件
% 它内部默认执行最强大的公共子表达式消除 (Common Subexpression Elimination)
matlabFunction(dJ_dv, dJ_da, dJ_dj, ...
    'File', 'backward_gradients.m', ...
    'Vars', vars_list, ...
    'Outputs', {'grad_v', 'grad_a', 'grad_j'});

fprintf('代码已成功生成并保存为当前目录下的 backward_gradients.m！\n');