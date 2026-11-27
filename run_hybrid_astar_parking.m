%% ========================================================================
%  run_hybrid_astar_parking.m
%  倒车入库 Hybrid A* 初始路径 (打开倒车, 小膨胀)
% ========================================================================
clear; clc; close all;
load('scenario_parking.mat');
veh=scenario.veh; map=scenario.map;

%% ---- 1. 小膨胀 (紧凑车位关键: 让圆能挤进去) ----
occMap = copy(map.occMap);
inflate(occMap, 0.2);              % 远小于车半宽, 只做基本安全余量

validator = validatorOccupancyMap;
validator.Map = occMap;
validator.ValidationDistance = 0.05;

%% ---- 2. Hybrid A* 规划器 (打开倒车) ----
planner = plannerHybridAStar(validator, ...
    'MinTurningRadius',      veh.Rmin, ...
    'MotionPrimitiveLength', 1.0, ...      % 更短基元, 适应精细机动
    'NumMotionPrimitives',   5, ...
    'ForwardCost',           1.0, ...
    'ReverseCost',           1.5, ...      % 接近前进成本 -> 允许倒车
    'InterpolationDistance', 0.3);

%% ---- 3. 规划 ----
tic;
route = plan(planner, map.start, map.goal);
tHA = toc;
pathStates = route.States;
fprintf('Hybrid A* 完成: %d个路径点, 耗时 %.3f s\n', size(pathStates,1), tHA);

%% ---- 4. 可视化 ----
figure('Color','w','Position',[100 100 640 400]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]'); ylabel('Y [m]');
title('Hybrid A* Coarse Path (Reverse Parking)','FontName','Times New Roman','FontSize',13);
for i=1:map.nOb
    v=map.obs{i}; fill(v(:,1),v(:,2),[0.15 0.15 0.15],'EdgeColor','k','LineWidth',1.2);
end
hPath=plot(pathStates(:,1),pathStates(:,2),'.','Color',[0 0.3 0.85],'MarkerSize',9);
step=max(1,round(size(pathStates,1)/14));
for k=1:step:size(pathStates,1)
    draw_vehicle(pathStates(k,:),veh,[0 0.4 0]);
end
plot(map.start(1),map.start(2),'go','MarkerFaceColor','g','MarkerSize',7);
plot(map.goal(1),map.goal(2),'ro','MarkerFaceColor','r','MarkerSize',7);
legend(hPath,{'Hybrid A* path'},'FontName','Times New Roman','Location','northoutside','Box','off');

%% ---- 5. 存储 ----
scenario.hybridAstar.states=pathStates;
scenario.hybridAstar.time=tHA;
save('scenario_parking.mat','scenario');
fprintf('粗路径已保存\n');

%% ========================================================================
function h=draw_vehicle(pose,veh,color)
    x=pose(1);y=pose(2);th=pose(3);
    xu=veh.ego(1);yu=veh.ego(2);xl=veh.ego(3);yl=veh.ego(4);
    corners=[xu yu; xu -yl; -xl -yl; -xl yu; xu yu];
    R=[cos(th) -sin(th);sin(th) cos(th)];
    world=(R*corners')'+[x y];
    h=plot(world(:,1),world(:,2),'Color',color,'LineWidth',1.0);
end