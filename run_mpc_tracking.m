%% ========================================================================
%  run_mpc_tracking.m
%  线性 MPC 轨迹跟踪 (跟踪 H-OBCA 输出的时空轨迹)
%  Step1: 先跑通跟踪主体, 出实际轨迹 + 误差 (动画下一步再加)
% ========================================================================
clear; clc; close all;
addpath('D:\casadi');           % 换成你的路径
import casadi.*
load('scenario_simple.mat');
veh=scenario.veh; map=scenario.map; L=veh.L;

Xref = scenario.obca.X;         % 4x(N+1) 参考状态 [x;y;theta;v]
Uref = scenario.obca.U;         % 2xN     参考控制 [delta;a]
Ts   = scenario.obca.Ts;
Nsim = size(Uref,2);            % 仿真步数 = 参考控制步数

%% ---- MPC 参数 ----
Np = 10;                        % 预测时域 (prediction horizon)
Q  = diag([10, 10, 5, 1]);      % 状态跟踪权重 [x y theta v]
R  = diag([1, 1]);              % 控制权重 [delta a]
delta_max = atan(L/veh.Rmin);
a_max = 1.5; v_max = 6.0;

%% ---- 自行车模型 (符号, 用于线性化) ----
xs=MX.sym('x',4); us=MX.sym('u',2);
xnext = [ xs(1)+xs(4)*cos(xs(3))*Ts;
          xs(2)+xs(4)*sin(xs(3))*Ts;
          xs(3)+xs(4)/L*tan(us(1))*Ts;
          xs(4)+us(2)*Ts ];
f_dyn = Function('f',{xs,us},{xnext});
A_fun = Function('A',{xs,us},{jacobian(xnext,xs)});  % df/dx
B_fun = Function('B',{xs,us},{jacobian(xnext,us)});  % df/du

%% ---- 仿真: 滚动时域 ----
x_now = Xref(:,1);              % 从参考起点出发 (可故意加扰动测试鲁棒性)
% x_now = Xref(:,1) + [0.3;0.3;0.05;0];   % 想测抗扰就取消注释

X_actual = zeros(4, Nsim+1);  X_actual(:,1)=x_now;
U_applied = zeros(2, Nsim);

for k = 1:Nsim
    % 1. 取未来Np步参考 (末尾不足则用最后一个填充)
    idx = k:min(k+Np-1, size(Uref,2));
    xref_win = Xref(:, idx+1);   % 对应下一状态参考
    uref_win = Uref(:, idx);
    np = numel(idx);

    % 2. 构造QP (在参考点线性化)
    opti = Opti();
    dx = opti.variable(4, np+1);    % 状态偏差
    du = opti.variable(2, np);      % 控制偏差
    J = 0;
    opti.subject_to(dx(:,1) == x_now - Xref(:,k));  % 初始偏差
    for j = 1:np
        xr = Xref(:,k+j-1);  ur = Uref(:,k+j-1);
        Ad = full(A_fun(xr,ur));  Bd = full(B_fun(xr,ur));
        % 线性化模型的偏差递推
        opti.subject_to(dx(:,j+1) == Ad*dx(:,j) + Bd*du(:,j));
        J = J + dx(:,j+1)'*Q*dx(:,j+1) + du(:,j)'*R*du(:,j);
        % 控制绝对量约束: uref+du 在界内
        opti.subject_to(-delta_max <= uref_win(1,j)+du(1,j) <= delta_max);
        opti.subject_to(-a_max     <= uref_win(2,j)+du(2,j) <= a_max);
    end
    opti.minimize(J);
    opts=struct; opts.ipopt.print_level=0; opts.print_time=0;
    opti.solver('ipopt',opts);

    try
        sol = opti.solve();
        du0 = sol.value(du(:,1));
    catch
        du0 = [0;0];        % 求解失败则不修正
    end

    % 3. 只执行第一个控制 (参考控制 + MPC修正量)
    u_apply = Uref(:,k) + du0;
    u_apply(1) = max(min(u_apply(1),delta_max),-delta_max);
    u_apply(2) = max(min(u_apply(2),a_max),-a_max);
    U_applied(:,k) = u_apply;

    % 4. 用【非线性】模型推进真实状态
    x_now = full(f_dyn(x_now, u_apply));
    X_actual(:,k+1) = x_now;
end

%% ---- 跟踪误差 ----
err_pos = sqrt((X_actual(1,:)-Xref(1,:)).^2 + (X_actual(2,:)-Xref(2,:)).^2);
err_yaw = abs(X_actual(3,:)-Xref(3,:));
fprintf('=== MPC 跟踪误差 ===\n');
fprintf('  位置误差: RMS=%.4f m, Max=%.4f m\n', rms(err_pos), max(err_pos));
fprintf('  航向误差: RMS=%.4f rad, Max=%.4f rad\n', rms(err_yaw), max(err_yaw));

%% ---- 图1: 参考 vs 实际轨迹 ----
figure('Color','w','Position',[100 100 950 260]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]'); ylabel('Y [m]');
title('MPC Trajectory Tracking','FontName','Times New Roman','FontSize',14);
for i=1:map.nOb
    v=map.obs{i}; fill(v(:,1),v(:,2),[0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
end
h1=plot(Xref(1,:),Xref(2,:),'--','Color',[0 0.3 0.85],'LineWidth',2);
h2=plot(X_actual(1,:),X_actual(2,:),'-','Color',[0.85 0 0],'LineWidth',1.5);
plot(map.start(1),map.start(2),'go','MarkerFaceColor','g','MarkerSize',7);
plot(map.goal(1),map.goal(2),'ro','MarkerFaceColor','r','MarkerSize',7);
legend([h1 h2],{'Reference (H-OBCA)','Actual (MPC)'}, ...
    'FontName','Times New Roman','Location','northoutside', ...
    'Orientation','horizontal','Box','off');

%% ---- 图2: 跟踪误差曲线 ----
figure('Color','w','Position',[100 100 700 300]);
subplot(2,1,1);
plot(0:Nsim, err_pos,'-','Color',[0.85 0 0],'LineWidth',1.5); box on;
set(gca,'FontName','Times New Roman'); ylabel('Position error [m]');
title('Tracking Error','FontName','Times New Roman');
subplot(2,1,2);
plot(0:Nsim, err_yaw,'-','Color',[0 0.3 0.85],'LineWidth',1.5); box on;
set(gca,'FontName','Times New Roman'); xlabel('Time step k'); ylabel('Heading error [rad]');

%% ---- 存储 ----
mpc.X_actual=X_actual; mpc.U_applied=U_applied;
mpc.err_pos=err_pos; mpc.err_yaw=err_yaw; mpc.Np=Np;
scenario.mpc=mpc;
save('scenario_simple.mat','scenario');
fprintf('MPC跟踪结果已保存\n');