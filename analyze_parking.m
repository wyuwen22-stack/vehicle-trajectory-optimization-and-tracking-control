%% ========================================================================
%  analyze_parking.m  泊车场景指标分析
% ========================================================================
clear; clc; close all;
load('scenario_parking.mat');
veh=scenario.veh; map=scenario.map; L=veh.L;
X=scenario.obca.X; U=scenario.obca.U; N=scenario.obca.N;
ref=scenario.obca.ref;
haPath=scenario.hybridAstar.states;

%% ---- 指标1: 曲率对比 ----
kappa_obca=path_curvature(X(1,:)',X(2,:)');
kappa_ha  =path_curvature(haPath(:,1),haPath(:,2));
figure('Color','w','Position',[100 100 600 280]); hold on; box on;
set(gca,'FontName','Times New Roman','FontSize',12);
s_o=[0;cumsum(sqrt(diff(X(1,:)').^2+diff(X(2,:)').^2))];
s_h=[0;cumsum(sqrt(diff(haPath(:,1)).^2+diff(haPath(:,2)).^2))];
plot(s_h,abs(kappa_ha),'.-','Color',[0 0.3 0.85],'MarkerSize',8);
plot(s_o,abs(kappa_obca),'-','Color',[0.85 0 0],'LineWidth',1.8);
xlabel('Path length s [m]'); ylabel('|Curvature| [1/m]');
title('Curvature Comparison (Parking)','FontName','Times New Roman');
legend({'Hybrid A*','H-OBCA'},'Box','off');
fprintf('曲率: HA max=%.3f std=%.3f | OBCA max=%.3f std=%.3f\n',...
    max(abs(kappa_ha)),std(abs(kappa_ha)),max(abs(kappa_obca)),std(abs(kappa_obca)));

%% ---- 指标2: 车身-障碍物最小距离 (王牌图) ----
dist=zeros(N+1,1);
for k=1:N+1
    dist(k)=veh_obs_dist(X(1:3,k)',veh,map.obs);
end
figure('Color','w','Position',[100 100 600 280]); hold on; box on;
set(gca,'FontName','Times New Roman','FontSize',12);
plot(0:N,dist,'-o','Color',[0.85 0 0],'LineWidth',1.5,'MarkerSize',4);
yline(0,'k--','Contact','LineWidth',1);
xlabel('Time step k'); ylabel('Min clearance to obstacle [m]');
title('Vehicle-Obstacle Clearance (Parking)','FontName','Times New Roman');
fprintf('最小间隙: %.3f m (全程>=0 无碰撞)\n', min(dist));

%% ---- 指标3: 控制量与约束 ----
delta=U(1,:); a=U(2,:); v=X(4,:);
delta_max=atan(L/veh.Rmin); v_max=3.0; a_max=1.0;
figure('Color','w','Position',[100 100 900 480]);
subplot(2,2,1);plot(0:N,v,'r','LineWidth',1.5);hold on;box on;yline(v_max,'k--');yline(-v_max,'k--');
set(gca,'FontName','Times New Roman');xlabel('k');ylabel('v [m/s]');title('Velocity (negative = reverse)');
subplot(2,2,2);plot(1:N,delta,'r','LineWidth',1.5);hold on;box on;yline(delta_max,'k--');yline(-delta_max,'k--');
set(gca,'FontName','Times New Roman');xlabel('k');ylabel('\delta [rad]');title('Steering angle');
subplot(2,2,3);plot(1:N,a,'r','LineWidth',1.5);hold on;box on;yline(a_max,'k--');yline(-a_max,'k--');
set(gca,'FontName','Times New Roman');xlabel('k');ylabel('a [m/s^2]');title('Acceleration');
subplot(2,2,4);plot(2:N,diff(delta),'r','LineWidth',1.5);box on;
set(gca,'FontName','Times New Roman');xlabel('k');ylabel('\Delta\delta');title('Steering rate');
fprintf('约束: max|v|=%.2f(%.2f) max|d|=%.3f(%.3f) max|a|=%.2f(%.2f)\n',...
    max(abs(v)),v_max,max(abs(delta)),delta_max,max(abs(a)),a_max);

%% ========================================================================
function k=path_curvature(x,y)
    n=length(x);k=zeros(n,1);
    for i=2:n-1
        dx1=x(i)-x(i-1);dy1=y(i)-y(i-1);dx2=x(i+1)-x(i);dy2=y(i+1)-y(i);
        a=hypot(dx1,dy1);b=hypot(dx2,dy2);c=hypot(x(i+1)-x(i-1),y(i+1)-y(i-1));
        ar=0.5*abs(dx1*dy2-dx2*dy1);
        if a*b*c>1e-9,k(i)=4*ar/(a*b*c);else,k(i)=0;end
    end
    k(1)=k(2);k(end)=k(end-1);
end
function d=veh_obs_dist(pose,veh,obs)
    xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
    corners=[xu yu;xu -yl;-xl -yl;-xl yu];
    th=pose(3);R=[cos(th) -sin(th);sin(th) cos(th)];
    vc=(R*corners')'+[pose(1) pose(2)];
    d=inf;
    for m=1:numel(obs)
        ob=obs{m}(1:end-1,:);
        for i=1:4, d=min(d,pt_poly(vc(i,:),ob)); end
        if any(inpolygon(vc(:,1),vc(:,2),ob(:,1),ob(:,2))), d=-abs(d); end
    end
end
function d=pt_poly(p,poly)
    n=size(poly,1);d=inf;
    for i=1:n
        a=poly(i,:);b=poly(mod(i,n)+1,:);
        ab=b-a;t=max(0,min(1,dot(p-a,ab)/max(dot(ab,ab),1e-9)));
        d=min(d,norm(p-(a+t*ab)));
    end
end