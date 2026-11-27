%% ========================================================================
%  run_parking_animation.m  倒车入库 MPC跟踪 + GIF动画
% ========================================================================
clear; clc; close all;
addpath('D:\casadi');
import casadi.*
load('scenario_parking.mat');
veh=scenario.veh; map=scenario.map; L=veh.L;
Xref=scenario.obca.X; Uref=scenario.obca.U; Ts=scenario.obca.Ts;
Nsim=size(Uref,2);

Np=10; Q=diag([15,15,8,1]); R=diag([1,1]);
delta_max=atan(L/veh.Rmin); a_max=1.0;

xs=MX.sym('x',4); us=MX.sym('u',2);
xnext=[xs(1)+xs(4)*cos(xs(3))*Ts; xs(2)+xs(4)*sin(xs(3))*Ts;
       xs(3)+xs(4)/L*tan(us(1))*Ts; xs(4)+us(2)*Ts];
f_dyn=Function('f',{xs,us},{xnext});
A_fun=Function('A',{xs,us},{jacobian(xnext,xs)});
B_fun=Function('B',{xs,us},{jacobian(xnext,us)});

x_now=Xref(:,1)+[0.3;0.3;0.05;0];
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

%% ---- GIF ----
gifName='parking_tracking.gif'; delayTime=0.12;
fig=figure('Color','w','Position',[100 100 640 420]);
for k=1:Nsim+1
    clf; hold on; axis equal; axis(map.XYbounds); box on;
    set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
    xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('Reverse Parking  (step %d/%d)',k-1,Nsim),...
          'FontName','Times New Roman','FontSize',13);
    for i=1:map.nOb
        v=map.obs{i}; fill(v(:,1),v(:,2),[0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
    end
    plot(Xref(1,:),Xref(2,:),'--','Color',[0 0.3 0.85],'LineWidth',1.5);
    plot(X_actual(1,1:k),X_actual(2,1:k),'-','Color',[0.85 0 0],'LineWidth',1.8);
    draw_vehicle(X_actual(1:3,k)',veh,[0 0.4 0]);
    plot(X_actual(1,k),X_actual(2,k),'o','Color',[0.85 0 0],'MarkerFaceColor',[0.85 0 0],'MarkerSize',5);
    plot(map.start(1),map.start(2),'go','MarkerFaceColor','g','MarkerSize',7);
    plot(map.goal(1),map.goal(2),'ro','MarkerFaceColor','r','MarkerSize',7);
    legend({'','Reference','Actual','Vehicle'},'FontName','Times New Roman',...
        'Location','northoutside','Orientation','horizontal','Box','off');
    drawnow;
    frame=getframe(fig); im=frame2im(frame); [Ag,mp]=rgb2ind(im,256);
    if k==1
        imwrite(Ag,mp,gifName,'gif','LoopCount',Inf,'DelayTime',delayTime);
    else
        imwrite(Ag,mp,gifName,'gif','WriteMode','append','DelayTime',delayTime);
    end
end
fprintf('GIF已导出: %s\n',fullfile(pwd,gifName));

%% ========================================================================
function draw_vehicle(pose,veh,color)
    x=pose(1);y=pose(2);th=pose(3);
    xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
    corners=[xu yu;xu -yl;-xl -yl;-xl yu;xu yu];
    R=[cos(th) -sin(th);sin(th) cos(th)];
    world=(R*corners')'+[x y];
    plot(world(:,1),world(:,2),'Color',color,'LineWidth',1.8);
end