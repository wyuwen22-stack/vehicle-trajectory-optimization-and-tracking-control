%% ========================================================================
%  run_obca_parking.m  (干净版)
%  倒车入库 H-OBCA
% ========================================================================
clear; clc; close all;
addpath('D:\casadi');           % 换成你的路径
import casadi.*
load('scenario_parking.mat');
veh=scenario.veh; map=scenario.map; L=veh.L;

%% ---- 1. warm start (含倒车速度符号判断) ----
path=scenario.hybridAstar.states;
Ndes=50;
idx=round(linspace(1,size(path,1),Ndes+1));
ref=path(idx,:);
N=size(ref,1)-1;
Ts=0.4;

xWS=zeros(4,N+1); xWS(1:3,:)=ref';
for k=1:N
    d=ref(k+1,1:2)-ref(k,1:2);
    heading=[cos(ref(k,3)),sin(ref(k,3))];
    s=sign(d*heading'); if s==0, s=1; end
    xWS(4,k)=s*norm(d)/Ts;
end
uWS=zeros(2,N);
for k=1:N
    dth=ref(k+1,3)-ref(k,3);
    dth=atan2(sin(dth),cos(dth));
    uWS(1,k)=atan(dth*L/max(abs(xWS(4,k))*Ts,0.1))*sign(xWS(4,k)+eps);
    uWS(2,k)=(xWS(4,min(k+1,N))-xWS(4,k))/Ts;
end

%% ---- 2. 物理限制 ----
v_max=3.0; a_max=1.0; delta_max=atan(L/veh.Rmin); ddelta_max=0.6;

%% ---- 3. 车身多面体 ----
xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
G=[1 0;-1 0;0 1;0 -1]; g=[xu;xl;yu;yl]; nG=4;

%% ---- 4. 障碍物H-rep ----
nOb=map.nOb; Aob=map.Aineq; bob=map.bineq;
nA=zeros(nOb,1); for m=1:nOb, nA(m)=size(Aob{m},1); end
dmin=0.05;

%% ---- 5. NLP ----
opti=Opti();
X=opti.variable(4,N+1); U=opti.variable(2,N);
LAM=cell(nOb,1); MU=cell(nOb,1);
for m=1:nOb
    LAM{m}=opti.variable(nA(m),N+1);
    MU{m} =opti.variable(nG,   N+1);
end

% 目标函数
J=0;
for k=1:N
    J=J+0.5*U(1,k)^2+0.1*U(2,k)^2;
    if k<N
        J=J+0.2*(U(1,k+1)-U(1,k))^2+2.0*(U(2,k+1)-U(2,k))^2;
    end
    for k=1:N+1
    J=J+0.3*((X(1,k)-xWS(1,k))^2+(X(2,k)-xWS(2,k))^2);  % 惩罚偏离初值位置
    end
    for k=1:N-1
    J=J+1.0*(X(4,k+1)-X(4,k))^2;
    end
end
opti.minimize(J);

% 运动学
for k=1:N
    opti.subject_to(X(1,k+1)==X(1,k)+X(4,k)*cos(X(3,k))*Ts);
    opti.subject_to(X(2,k+1)==X(2,k)+X(4,k)*sin(X(3,k))*Ts);
    opti.subject_to(X(3,k+1)==X(3,k)+X(4,k)/L*tan(U(1,k))*Ts);
    opti.subject_to(X(4,k+1)==X(4,k)+U(2,k)*Ts);
end

% 边界: 起点硬约束, 终点软约束
opti.subject_to(X(1:3,1)==map.start');   opti.subject_to(X(4,1)==0);
opti.subject_to(map.goal(1)-0.05 <= X(1,N+1) <= map.goal(1)+0.05);
opti.subject_to(map.goal(2)-0.05 <= X(2,N+1) <= map.goal(2)+0.05);
opti.subject_to(map.goal(3)-0.05 <= X(3,N+1) <= map.goal(3)+0.05);
opti.subject_to(-0.1 <= X(4,N+1) <= 0.1);

% 物理限制
opti.subject_to(-v_max<=X(4,:)<=v_max);
opti.subject_to(-delta_max<=U(1,:)<=delta_max);
opti.subject_to(-a_max<=U(2,:)<=a_max);
for k=1:N-1
    opti.subject_to(-ddelta_max*Ts<=U(1,k+1)-U(1,k)<=ddelta_max*Ts);
end

% 场地边界约束
opti.subject_to(map.XYbounds(1)+1 <= X(1,:) <= map.XYbounds(2)-1);
opti.subject_to(map.XYbounds(3)   <= X(2,:) <= map.XYbounds(4)-1);

% 对偶碰撞约束
for k=1:N+1
    th=X(3,k); R=[cos(th) -sin(th);sin(th) cos(th)]; t=[X(1,k);X(2,k)];
    for m=1:nOb
        A=Aob{m}; b=bob{m}; lam=LAM{m}(:,k); mu=MU{m}(:,k);
        opti.subject_to(lam>=0); opti.subject_to(mu>=0);
        opti.subject_to(-(g'*mu)+(A*t-b)'*lam>=dmin);
        opti.subject_to(G'*mu+R'*(A'*lam)==0);
        opti.subject_to((A'*lam)'*(A'*lam)<=1);
    end
end

%% ---- 6. warm start ----
opti.set_initial(X,xWS); opti.set_initial(U,uWS);
for m=1:nOb
    opti.set_initial(LAM{m},0.1*ones(nA(m),N+1));
    opti.set_initial(MU{m}, 0.1*ones(nG,N+1));
end

%% ---- 7. 求解 ----
opts=struct; opts.ipopt.print_level=5; opts.ipopt.max_iter=8000;
opts.ipopt.tol=1e-4; opts.print_time=true;
opti.solver('ipopt',opts);
try
    sol=opti.solve(); Xopt=sol.value(X); Uopt=sol.value(U);
    fprintf('\n=== 泊车 H-OBCA 求解成功 ===\n');
catch ME
    fprintf('\n=== 求解失败, 输出调试值 ===\n%s\n',ME.message);
    Xopt=opti.debug.value(X); Uopt=opti.debug.value(U);
end

%% ---- 8. 可视化 ----
figure('Color','w','Position',[100 100 640 400]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]'); ylabel('Y [m]');
title('H-OBCA Reverse Parking','FontName','Times New Roman','FontSize',13);
for i=1:nOb
    v=map.obs{i}; fill(v(:,1),v(:,2),[0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
end
h1=plot(ref(:,1),ref(:,2),'.','Color',[0 0.3 0.85],'MarkerSize',8);
h2=plot(Xopt(1,:),Xopt(2,:),'-','Color',[0.85 0 0],'LineWidth',1.8);
% 密集墨绿色车身叠影
for k=1:N+1                              % 每个点都画, 最密集
    draw_vehicle(Xopt(1:3,k)',veh,[0 0.4 0]);   % 墨绿色 RGB
end
plot(map.start(1),map.start(2),'go','MarkerFaceColor','g','MarkerSize',7);
plot(map.goal(1),map.goal(2),'ro','MarkerFaceColor','r','MarkerSize',7);
legend([h1 h2],{'Hybrid A* (warm start)','H-OBCA optimized'}, ...
    'FontName','Times New Roman','Location','northoutside', ...
    'Orientation','horizontal','Box','off');

%% ---- 9. 存储 ----
obca.X=Xopt; obca.U=Uopt; obca.Ts=Ts; obca.N=N; obca.ref=ref;
scenario.obca=obca;
save('scenario_parking.mat','scenario');
fprintf('泊车H-OBCA结果已保存\n');

%% ========================================================================
function h=draw_vehicle(pose,veh,color)
    x=pose(1);y=pose(2);th=pose(3);
    xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
    corners=[xu yu; xu -yl; -xl -yl; -xl yu; xu yu];
    R=[cos(th) -sin(th);sin(th) cos(th)];
    world=(R*corners')'+[x y];
    h=plot(world(:,1),world(:,2),'Color',color,'LineWidth',1.0);
end