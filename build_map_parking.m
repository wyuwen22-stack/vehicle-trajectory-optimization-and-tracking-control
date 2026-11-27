%% ========================================================================
%  build_map_parking.m
%  倒车入库场景 —— 2D地图建模 (参数来自 Zhang H-OBCA, reverse parking)
% ========================================================================
clear; clc; close all;

%% ---- 1. 车辆参数 (与简单场景一致, 来自Zhang) ----
veh.L    = 2.7;
veh.ego  = [3.7, 1.0, 1.0, 1.0];   % 车长4.7, 车宽2.0, 后轴中心参考
veh.Rmin = 5.0;

%% ---- 2. 场地边界 + 起终点 [x y theta] (来自Zhang) ----
map.XYbounds = [-15, 15, 0, 12];   % 显示范围
map.start = [-9, 7, pi];          % 上方通道, 车头朝右
map.goal  = [ 0, 1.3, pi/2];       % 倒入下方车位, 车头朝上

%% ---- 3. 障碍物 (闭合矩形, 顶点表示) ----
% obs1=左侧停车, obs2=右侧停车, obs3=上方墙; 车位开口 x∈[-1.3,1.3]
obs{1} = [-12,0; -1.3,0; -1.3,5; -12,5; -12,0];   % 左车块
obs{2} = [ 1.3,0;  12,0;  12,5;  1.3,5;  1.3,0];   % 右车块
obs{3} = [-12,11;  12,11; 12,12; -12,12; -12,11];  % 上墙 11 -> 13;  % 上墙
map.obs = obs;
map.nOb = numel(obs);

%% ---- 4. 顶点 -> H-representation (供OBCA) ----
map.Aineq = cell(map.nOb,1); map.bineq = cell(map.nOb,1);
for i=1:map.nOb
    [A,b] = poly2hrep(obs{i});
    map.Aineq{i}=A; map.bineq{i}=b;
end

%% ---- 5. 占据栅格地图 (供Hybrid A*, 支持负坐标) ----
res = 5;
xmin=map.XYbounds(1); xmax=map.XYbounds(2);
ymin=map.XYbounds(3); ymax=map.XYbounds(4);
W = xmax-xmin; H = ymax-ymin;
occMap = binaryOccupancyMap(W, H, res);
occMap.GridLocationInWorld = [xmin, ymin];   % 平移原点支持负坐标
[xg,yg] = meshgrid(linspace(xmin,xmax,W*res), linspace(ymin,ymax,H*res));
occ = false(size(xg));
for i=1:map.nOb
    v=obs{i}; occ = occ | inpolygon(xg,yg,v(:,1),v(:,2));
end
setOccupancy(occMap,[xg(occ),yg(occ)],1);
map.occMap = occMap;

%% ---- 6. 可视化 ----
figure('Color','w','Position',[100 100 640 400]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]'); ylabel('Y [m]');
title('Reverse Parking Scenario','FontName','Times New Roman','FontSize',14);

hObs=[];
for i=1:map.nOb
    v=obs{i};
    hObs=fill(v(:,1),v(:,2),[0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
end
hStart=draw_vehicle(map.start,veh,[0 0.5 0]);
hGoal =draw_vehicle(map.goal, veh,[0.85 0 0]);
plot(map.start(1),map.start(2),'o','Color',[0 0.5 0],'MarkerFaceColor',[0 0.5 0],'MarkerSize',6);
hRef=plot(map.goal(1),map.goal(2),'o','Color',[0.85 0 0],'MarkerFaceColor',[0.85 0 0],'MarkerSize',6);
text(map.start(1),map.start(2)+0.8,'Start','Color',[0 0.5 0],'FontName','Times New Roman','FontWeight','bold');
text(map.goal(1)+1,map.goal(2),'Goal','Color',[0.85 0 0],'FontName','Times New Roman','FontWeight','bold');

legend([hObs,hStart,hGoal,hRef], ...
   {'Obstacle','Start pose','Goal pose','Rear-axle ref.'}, ...
   'FontName','Times New Roman','FontSize',9,'Location','northoutside', ...
   'Orientation','horizontal','Box','off');

%% ---- 7. 打包保存 ----
scenario.veh=veh; scenario.map=map;
save('scenario_parking.mat','scenario');
fprintf('泊车场景建模完成: %d个障碍物, 车位开口2.6m, 已保存\n', map.nOb);

%% ========================================================================
function [A,b]=poly2hrep(V)
    V=V(1:end-1,:); n=size(V,1); c=mean(V,1);
    A=zeros(n,2); b=zeros(n,1);
    for k=1:n
        p1=V(k,:); p2=V(mod(k,n)+1,:);
        e=p2-p1; nrm=[e(2),-e(1)]; nrm=nrm/norm(nrm);
        if nrm*(c-p1)'>0, nrm=-nrm; end
        A(k,:)=nrm; b(k)=nrm*p1';
    end
end
function h=draw_vehicle(pose,veh,color)
    x=pose(1);y=pose(2);th=pose(3);
    xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
    corners=[xu yu; xu -yl; -xl -yl; -xl yu; xu yu];
    R=[cos(th) -sin(th);sin(th) cos(th)];
    world=(R*corners')'+[x y];
    h=plot(world(:,1),world(:,2),'Color',color,'LineWidth',1.8);
end