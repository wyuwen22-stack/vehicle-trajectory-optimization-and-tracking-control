%% ========================================================================
%  run_mpc_parking.m  倒车入库 MPC 跟踪 (含倒车)
% ========================================================================
clear; clc; close all;
addpath('D:\casadi');
import casadi.*
load('scenario_parking.mat');
veh=scenario.veh; map=scenario.map; L=veh.L;
Xref=scenario.obca.X; Uref=scenario.obca.U; Ts=scenario.obca.Ts;
Nsim=size(Uref,2);

%% ---- MPC参数 ----
Np=10; Q=diag([15,15,8,1]); R=diag([1,1]);
delta_max=atan(L/veh.Rmin); a_max=1.0; v_max=3.0;

%% ---- 模型 ----
xs=MX.sym('x',4); us=MX.sym('u',2);
xnext=[xs(1)+xs(4)*cos(xs(3))*Ts; xs(2)+xs(4)*sin(xs(3))*Ts;
       xs(3)+xs(4)/L*tan(us(1))*Ts; xs(4)+us(2)*Ts];
f_dyn=Function('f',{xs,us},{xnext});
A_fun=Function('A',{xs,us},{jacobian(xnext,xs)});
B_fun=Function('B',{xs,us},{jacobian(xnext,us)});

%% ---- 仿真 (带初始扰动, 展示纠偏) ----
x_now=Xref(:,1)+[0.3;0.3;0.05;0];   % 初始偏离
X_actual=zeros(4,Nsim+1); X_actual(:,1)=x_now;

for k=1:Nsim
    idx=k:min(k+Np-1,size(Uref,2)); np=numel(idx);
    uref_win=Uref(:,idx);
    opti=Opti(); dx=opti.variable(4,np+1); du=opti.variable(2,np); J=0;
    opti.subject_to(dx(:,1)==x_now-Xref(:,k));
    for j=1:np
        xr=Xref(:,k+j-1); ur=Uref(:,k+j-1);
        Ad=full(A_fun(xr,ur)); Bd=full(B_fun(xr,ur));
        opti.subject_to(dx(:,j+1)==Ad*dx(:,j)+Bd*du(:,j));
        J=J+dx(:,j+1)'*Q*dx(:,j+1)+du(:,j)'*R*du(:,j);
        opti.subject_to(-delta_max<=uref_win(1,j)+du(1,j)<=delta_max);
        opti.subject_to(-a_max<=uref_win(2,j)+du(2,j)<=a_max);
    end
    opti.minimize(J);
    opts=struct;opts.ipopt.print_level=0;opts.print_time=0;
    opti.solver('ipopt',opts);
    try sol=opti.solve(); du0=sol.value(du(:,1)); catch, du0=[0;0]; end
    u_apply=Uref(:,k)+du0;
    u_apply(1)=max(min(u_apply(1),delta_max),-delta_max);
    u_apply(2)=max(min(u_apply(2),a_max),-a_max);
    x_now=full(f_dyn(x_now,u_apply));
    X_actual(:,k+1)=x_now;
end

%% ---- 误差 ----
err_pos=sqrt((X_actual(1,:)-Xref(1,:)).^2+(X_actual(2,:)-Xref(2,:)).^2);
err_yaw=abs(atan2(sin(X_actual(3,:)-Xref(3,:)),cos(X_actual(3,:)-Xref(3,:))));
fprintf('=== 泊车MPC跟踪误差 ===\n');
fprintf('  位置: RMS=%.4f m, Max=%.4f m\n',rms(err_pos),max(err_pos));
fprintf('  航向: RMS=%.4f rad, Max=%.4f rad\n',rms(err_yaw),max(err_yaw));

%% ---- 图1: 轨迹 ----
figure('Color','w','Position',[100 100 640 400]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]'); ylabel('Y [m]');
title('MPC Tracking (Reverse Parking)','FontName','Times New Roman','FontSize',13);
for i=1:map.nOb
    v=map.obs{i}; fill(v(:,1),v(:,2),[0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
end
h1=plot(Xref(1,:),Xref(2,:),'--','Color',[0 0.3 0.85],'LineWidth',2);
h2=plot(X_actual(1,:),X_actual(2,:),'-','Color',[0.85 0 0],'LineWidth',1.5);
plot(map.start(1),map.start(2),'go','MarkerFaceColor','g','MarkerSize',7);
plot(map.goal(1),map.goal(2),'ro','MarkerFaceColor','r','MarkerSize',7);
legend([h1 h2],{'Reference (H-OBCA)','Actual (MPC)'},...
    'FontName','Times New Roman','Location','northoutside','Orientation','horizontal','Box','off');

%% ---- 图2: 误差 ----
figure('Color','w','Position',[100 100 640 300]);
subplot(2,1,1);plot(0:Nsim,err_pos,'r','LineWidth',1.5);box on;
set(gca,'FontName','Times New Roman');ylabel('Pos error [m]');title('Tracking Error (Parking)');
subplot(2,1,2);plot(0:Nsim,err_yaw,'b','LineWidth',1.5);box on;
set(gca,'FontName','Times New Roman');xlabel('k');ylabel('Yaw error [rad]');

%% ---- 存储 ----
mpc.X_actual=X_actual; mpc.err_pos=err_pos; mpc.err_yaw=err_yaw;
scenario.mpc=mpc;
save('scenario_parking.mat','scenario');
fprintf('泊车MPC结果已保存\n');
figure('Color','w'); hold on; box on;
plot(0:size(Xref,2)-1, Xref(4,:),'b-','LineWidth',1.5);
plot(0:size(X_actual,2)-1, X_actual(4,:),'r--','LineWidth',1.5);
set(gca,'FontName','Times New Roman');
xlabel('k'); ylabel('v [m/s]');
legend({'Reference v','Actual v'},'Box','off');
title('Velocity profile');
yline(0,'k:');