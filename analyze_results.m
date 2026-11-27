%% ========================================================================
%  analyze_results.m
%  OBCA vs Hybrid A* 定量分析: 平滑性 / 安全性 / 动力学可行性
% ========================================================================
clear; clc; close all;
load('scenario_simple.mat');
veh=scenario.veh; map=scenario.map;
L=veh.L;
X=scenario.obca.X;  U=scenario.obca.U;  Ts=scenario.obca.Ts;  N=scenario.obca.N;
ref=scenario.obca.ref;                      % Hybrid A* warm start [x y theta]
haPath=scenario.hybridAstar.states;         % Hybrid A* 原始密集路径

%% ------------------------------------------------------------------------
%  指标1: 平滑性 —— 曲率对比
% ------------------------------------------------------------------------
kappa_obca = path_curvature(X(1,:)', X(2,:)');
kappa_ha   = path_curvature(haPath(:,1), haPath(:,2));

figure('Color','w','Position',[100 100 700 300]); hold on; box on;
set(gca,'FontName','Times New Roman','FontSize',12);
s_obca=[0;cumsum(sqrt(diff(X(1,:)').^2+diff(X(2,:)').^2))];
s_ha  =[0;cumsum(sqrt(diff(haPath(:,1)).^2+diff(haPath(:,2)).^2))];
plot(s_ha,  abs(kappa_ha),  '.-','Color',[0 0.3 0.85],'LineWidth',1.2,'MarkerSize',10);
plot(s_obca,abs(kappa_obca),'-', 'Color',[0.85 0 0],'LineWidth',1.8);
xlabel('Path length s [m]'); ylabel('|Curvature| \kappa [1/m]');
title('Curvature Comparison','FontName','Times New Roman');
legend({'Hybrid A*','OBCA'},'Box','off');

fprintf('=== 平滑性 (曲率) ===\n');
fprintf('  Hybrid A*: max|k|=%.3f, std|k|=%.3f\n',max(abs(kappa_ha)),std(abs(kappa_ha)));
fprintf('  OBCA     : max|k|=%.3f, std|k|=%.3f\n',max(abs(kappa_obca)),std(abs(kappa_obca)));

%% ------------------------------------------------------------------------
%  指标2: 安全性 —— 车身到最近障碍物的最小距离
% ------------------------------------------------------------------------
dist_obca = zeros(N+1,1);
for k=1:N+1
    dist_obca(k)=vehicle_obstacle_dist(X(1:3,k)', veh, map.obs);
end
figure('Color','w','Position',[100 100 700 300]); hold on; box on;
set(gca,'FontName','Times New Roman','FontSize',12);
plot(0:N, dist_obca,'-o','Color',[0.85 0 0],'LineWidth',1.5,'MarkerSize',4);
yline(0,'k--','Contact','LineWidth',1);
xlabel('Time step k'); ylabel('Min distance to obstacle [m]');
title('Vehicle-Obstacle Clearance (OBCA)','FontName','Times New Roman');

fprintf('=== 安全性 (车身-障碍物最小距离) ===\n');
fprintf('  全程最小间隙: %.3f m (>=0 表示无碰撞)\n', min(dist_obca));

%% ------------------------------------------------------------------------
%  指标3: 动力学可行性 —— 控制量与约束
% ------------------------------------------------------------------------
delta=U(1,:); a=U(2,:); v=X(4,:);
delta_max=atan(L/veh.Rmin); v_max=6.0; a_max=1.5;

figure('Color','w','Position',[100 100 900 500]);
subplot(2,2,1); plot(0:N,v,'-','Color',[0.85 0 0],'LineWidth',1.5); hold on; box on;
yline(v_max,'k--'); yline(-v_max,'k--');
set(gca,'FontName','Times New Roman'); xlabel('k'); ylabel('v [m/s]'); title('Velocity');

subplot(2,2,2); plot(1:N,delta,'-','Color',[0.85 0 0],'LineWidth',1.5); hold on; box on;
yline(delta_max,'k--'); yline(-delta_max,'k--');
set(gca,'FontName','Times New Roman'); xlabel('k'); ylabel('\delta [rad]'); title('Steering angle');

subplot(2,2,3); plot(1:N,a,'-','Color',[0.85 0 0],'LineWidth',1.5); hold on; box on;
yline(a_max,'k--'); yline(-a_max,'k--');
set(gca,'FontName','Times New Roman'); xlabel('k'); ylabel('a [m/s^2]'); title('Acceleration');

subplot(2,2,4); plot(2:N,diff(delta),'-','Color',[0.85 0 0],'LineWidth',1.5); box on;
set(gca,'FontName','Times New Roman'); xlabel('k'); ylabel('\Delta\delta [rad]'); title('Steering rate');

fprintf('=== 动力学可行性 (约束满足性) ===\n');
fprintf('  max|v|=%.2f (lim %.2f) %s\n',max(abs(v)),v_max,check(max(abs(v)),v_max));
fprintf('  max|delta|=%.3f (lim %.3f) %s\n',max(abs(delta)),delta_max,check(max(abs(delta)),delta_max));
fprintf('  max|a|=%.2f (lim %.2f) %s\n',max(abs(a)),a_max,check(max(abs(a)),a_max));

%% ========================================================================
function k=path_curvature(x,y)
% 三点法数值曲率
    n=length(x); k=zeros(n,1);
    for i=2:n-1
        dx1=x(i)-x(i-1); dy1=y(i)-y(i-1);
        dx2=x(i+1)-x(i); dy2=y(i+1)-y(i);
        a=hypot(dx1,dy1); b=hypot(dx2,dy2);
        c=hypot(x(i+1)-x(i-1), y(i+1)-y(i-1));
        area=0.5*abs(dx1*dy2-dx2*dy1);
        if a*b*c>1e-9, k(i)=4*area/(a*b*c); else, k(i)=0; end
    end
    k(1)=k(2); k(end)=k(end-1);
end

function d=vehicle_obstacle_dist(pose,veh,obs)
% 车身矩形到所有障碍物的最小距离(负值=侵入)
    xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
    corners=[xu yu; xu -yl; -xl -yl; -xl yu];
    th=pose(3); R=[cos(th) -sin(th);sin(th) cos(th)];
    vc=(R*corners')'+[pose(1) pose(2)];   % 世界系车身四角
    d=inf;
    for m=1:numel(obs)
        ob=obs{m}(1:end-1,:);
        % 车角点到障碍物边的最小距离 (近似)
        for i=1:4
            d=min(d, point_poly_dist(vc(i,:), ob));
        end
        % 若车角在障碍物内 -> 负
        if any(inpolygon(vc(:,1),vc(:,2),ob(:,1),ob(:,2)))
            d=-abs(d);
        end
    end
end

function d=point_poly_dist(p,poly)
    n=size(poly,1); d=inf;
    for i=1:n
        a=poly(i,:); b=poly(mod(i,n)+1,:);
        d=min(d, seg_dist(p,a,b));
    end
end
function d=seg_dist(p,a,b)
    ab=b-a; t=max(0,min(1,dot(p-a,ab)/max(dot(ab,ab),1e-9)));
    d=norm(p-(a+t*ab));
end
function s=check(val,lim)
    if val<=lim*1.001, s='[OK]'; else, s='[VIOLATED]'; end
end