%% ========================================================================
%  run_hybrid_astar.m
%  第二步: Hybrid A* 规划粗路径 (作为OBCA的warm start)
% ========================================================================
clear; clc; close all;
load('scenario_simple.mat');        % 载入 scenario (veh + map)
veh = scenario.veh;
map = scenario.map;

%% ------------------------------------------------------------------------
%  1. 用车身尺寸膨胀地图 (Hybrid A*用圆/膨胀近似车身做碰撞检测)
% ------------------------------------------------------------------------
% 车身半宽约1.0m, 用膨胀把车近似成"点"在膨胀后的地图上搜索
occMap = copy(map.occMap);
inflate(occMap, 1.0);               % 膨胀半径≈车半宽, 体现圆近似的保守性

validator = validatorOccupancyMap;
validator.Map = occMap;
validator.ValidationDistance = 0.1;

%% ------------------------------------------------------------------------
%  2. 创建 Hybrid A* 规划器 (运动学参数来自车辆模型)
% ------------------------------------------------------------------------
planner = plannerHybridAStar(validator, ...
    'MinTurningRadius',      veh.Rmin, ...   % 最小转弯半径 5m
    'MotionPrimitiveLength', 2.0, ...        % 运动基元长度
    'NumMotionPrimitives',   5, ...
    'ForwardCost',           1.0, ...
    'ReverseCost',           3.0, ...        % 简单场景不鼓励倒车
    'InterpolationDistance', 0.5);

%% ------------------------------------------------------------------------
%  3. 规划
% ------------------------------------------------------------------------
startPose = map.start;              % [x y theta]
goalPose  = map.goal;

tic;
route = plan(planner, startPose, goalPose);
timeHybridAstar = toc;

pathStates = route.States;          % Nx3: [x y theta]
fprintf('Hybrid A* 完成: %d个路径点, 耗时 %.3f s\n', ...
        size(pathStates,1), timeHybridAstar);

%% ------------------------------------------------------------------------
%  4. 可视化: 地图 + 粗路径 (沿用Zhang风格)
% ------------------------------------------------------------------------
figure('Color','w','Position',[100 100 950 260]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]','FontName','Times New Roman','FontSize',13);
ylabel('Y [m]','FontName','Times New Roman','FontSize',13);
title('Hybrid A* Coarse Path','FontName','Times New Roman','FontSize',14);

% 障碍物
for i = 1:map.nOb
    v = map.obs{i};
    fill(v(:,1), v(:,2), [0.15 0.15 0.15], 'EdgeColor','k','LineWidth',1.2);
end

% 粗路径 (蓝色小点)
hPath = plot(pathStates(:,1), pathStates(:,2), '.', ...
             'Color',[0 0.3 0.85],'MarkerSize',10);
% 沿路径每隔几个点画车身矩形, 展示车身姿态
step = max(1, round(size(pathStates,1)/12));
for k = 1:step:size(pathStates,1)
    draw_vehicle(pathStates(k,:), veh, [0 0.4 0]);
end

% 起终点
plot(startPose(1),startPose(2),'go','MarkerFaceColor','g','MarkerSize',7);
plot(goalPose(1), goalPose(2), 'ro','MarkerFaceColor','r','MarkerSize',7);

legend(hPath,{'Hybrid A* path'},'FontName','Times New Roman', ...
       'FontSize',10,'Location','northoutside','Box','off');

%% ------------------------------------------------------------------------
%  5. 存储路径 (供OBCA warm start + 后续指标对比)
% ------------------------------------------------------------------------
hybridAstar.states = pathStates;        % [x y theta]
hybridAstar.time   = timeHybridAstar;
scenario.hybridAstar = hybridAstar;
save('scenario_simple.mat','scenario');
fprintf('粗路径已保存至 scenario_simple.mat\n');


%% ========================================================================
function h = draw_vehicle(pose, veh, color)
    x=pose(1); y=pose(2); th=pose(3);
    xu=veh.ego(1); yu=veh.ego(2); xl=veh.ego(3); yl=veh.ego(4);
    corners = [ xu,  yu;  xu, -yl; -xl, -yl; -xl, yu; xu, yu];
    R = [cos(th) -sin(th); sin(th) cos(th)];
    world = (R*corners')' + [x, y];
    h = plot(world(:,1), world(:,2), 'Color', color, 'LineWidth',1.2);
end