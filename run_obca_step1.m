addpath('D:\casadi');   % 换成你的实际路径
import casadi.*
%% ========================================================================
%  run_obca_step1.m
%  OBCA 第一步: 运动学+边界+物理约束 (暂不加碰撞约束)
%  目的: 验证建模骨架与CasADi/IPOPT求解链路
% ========================================================================
clear; clc; close all;
import casadi.*
load('scenario_simple.mat');
veh = scenario.veh;  map = scenario.map;
L   = veh.L;

%% ---- 1. 从 Hybrid A* 路径降采样, 构造 warm start ----
path = scenario.hybridAstar.states;     % Mx3 [x y theta]
Ndes = 40;                              % 期望的离散点数
idx  = round(linspace(1, size(path,1), Ndes+1));
ref  = path(idx, :);                    % (N+1)x3
N    = size(ref,1) - 1;

Ts   = 0.4;                             % 固定采样时间 [s]

% warm start: 状态
xWS = zeros(4, N+1);
xWS(1:3,:) = ref';
for k = 1:N
    d = norm(ref(k+1,1:2) - ref(k,1:2));
    xWS(4,k) = d / Ts;                  % 速度≈段长/Ts
end
% warm start: 控制
uWS = zeros(2, N);
for k = 1:N
    dth = ref(k+1,3) - ref(k,3);
    uWS(1,k) = atan(dth * L / max(xWS(4,k)*Ts, 0.1));  % 转角δ
    uWS(2,k) = (xWS(4,min(k+1,N)) - xWS(4,k)) / Ts;    % 加速度a
end

%% ---- 2. 物理限制 ----
v_max     = 6.0;                        % 最大速度 [m/s]
a_max     = 1.5;                        % 最大加速度 [m/s^2]
delta_max = atan(L/veh.Rmin);           % 最大前轮转角 (由Rmin反推)
ddelta_max= 0.6;                        % 转角变化率上限 [rad/s]

%% ---- 3. 建立 NLP (Opti) ----
opti = Opti();
X = opti.variable(4, N+1);              % [x;y;theta;v]
U = opti.variable(2, N);                % [delta;a]

% 3.1 目标函数: 控制量 + 控制变化率 (平滑)
J = 0;
for k = 1:N
    J = J + 0.1*U(1,k)^2 + 0.05*U(2,k)^2;      % 控制量
    if k < N
        J = J + 1.0*(U(1,k+1)-U(1,k))^2 ...    % 转角平滑
              + 0.5*(U(2,k+1)-U(2,k))^2;       % 加速度平滑
    end
end
opti.minimize(J);

% 3.2 运动学约束 (自行车模型, 后轴中心)
for k = 1:N
    xn = X(1,k) + X(4,k)*cos(X(3,k))*Ts;
    yn = X(2,k) + X(4,k)*sin(X(3,k))*Ts;
    tn = X(3,k) + X(4,k)/L*tan(U(1,k))*Ts;
    vn = X(4,k) + U(2,k)*Ts;
    opti.subject_to(X(1,k+1) == xn);
    opti.subject_to(X(2,k+1) == yn);
    opti.subject_to(X(3,k+1) == tn);
    opti.subject_to(X(4,k+1) == vn);
end

% 3.3 边界条件 (起点/终点, 起终速度为0)
opti.subject_to(X(1:3,1)   == map.start');
opti.subject_to(X(4,1)     == 0);
opti.subject_to(X(1:3,N+1) == map.goal');
opti.subject_to(X(4,N+1)   == 0);

% 3.4 物理限制
opti.subject_to(-v_max     <= X(4,:) <= v_max);
opti.subject_to(-delta_max <= U(1,:) <= delta_max);
opti.subject_to(-a_max     <= U(2,:) <= a_max);
for k = 1:N-1
    opti.subject_to(-ddelta_max*Ts <= U(1,k+1)-U(1,k) <= ddelta_max*Ts);
end

%% ---- 4. 设置 warm start + 求解器 ----
opti.set_initial(X, xWS);
opti.set_initial(U, uWS);

opts = struct;
opts.ipopt.print_level = 5;
opts.ipopt.max_iter    = 3000;
opts.print_time        = true;
opti.solver('ipopt', opts);

%% ---- 5. 求解 ----
try
    sol = opti.solve();
    Xopt = sol.value(X);
    Uopt = sol.value(U);
    fprintf('\n=== OBCA Step1 求解成功 ===\n');
catch ME
    fprintf('\n=== 求解失败, 输出调试初值 ===\n%s\n', ME.message);
    Xopt = opti.debug.value(X);
    Uopt = opti.debug.value(U);
end

%% ---- 6. 可视化: Hybrid A*(蓝点) vs OBCA(红线) ----
figure('Color','w','Position',[100 100 950 260]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]'); ylabel('Y [m]');
title('OBCA Step1 (no collision constraint yet)', ...
      'FontName','Times New Roman','FontSize',14);
for i = 1:map.nOb
    v = map.obs{i};
    fill(v(:,1), v(:,2), [0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
end
h1 = plot(ref(:,1), ref(:,2), '.', 'Color',[0 0.3 0.85],'MarkerSize',10);
h2 = plot(Xopt(1,:), Xopt(2,:), '-', 'Color',[0.85 0 0],'LineWidth',1.8);
plot(map.start(1),map.start(2),'go','MarkerFaceColor','g','MarkerSize',7);
plot(map.goal(1), map.goal(2), 'ro','MarkerFaceColor','r','MarkerSize',7);
legend([h1 h2],{'Hybrid A* (warm start)','OBCA optimized'}, ...
       'FontName','Times New Roman','Location','northoutside', ...
       'Orientation','horizontal','Box','off');

%% ---- 7. 存储 ----
obca.X = Xopt; obca.U = Uopt; obca.Ts = Ts; obca.N = N; obca.ref = ref;
scenario.obca_step1 = obca;
save('scenario_simple.mat','scenario');
fprintf('OBCA Step1 结果已保存\n');