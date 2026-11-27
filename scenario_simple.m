%% ========================================================================
%  build_map_simple.m
%  简单避障场景 —— 2D地图建模 (绕行任务)
%  沿用 Zhang(H-OBCA) 约定: 后轴中心为参考点, 单位米, 顶点+H-rep双表示
% ========================================================================
clear; clc; close all;

%% ------------------------------------------------------------------------
%  1. 车辆参数 (来自 Zhang main.jl)
% ------------------------------------------------------------------------
veh.L    = 2.7;                 % 轴距 [m]
% ego = [x_upper, y_upper, x_lower, y_lower], 相对后轴中心
% 车头方向伸 3.7, 车尾伸 1.0, 左右各 1.0  => 车长4.7, 车宽2.0
veh.ego  = [3.7, 1.0, 1.0, 1.0];
veh.Rmin = 5.0;                 % 最小转弯半径 [m] (简单场景先用标准值)

%% ------------------------------------------------------------------------
%  2. 场景边界 + 起终点 (后轴中心位姿 [x, y, theta])
% ------------------------------------------------------------------------
map.XYbounds = [0, 50, 0, 8];   % [x_min x_max y_min y_max] 一段50x8m通路
map.start    = [ 3, 2, 0];      % 左端, 车头朝右(+x), 位于下车道
map.goal     = [47, 2, 0];      % 右端, 同朝向, 回到下车道

%% ------------------------------------------------------------------------
%  3. 障碍物 (顶点表示, 顺时针, 首顶点在末尾重复 —— 与Zhang一致)
%     语义: obs1=前方抛锚车(挡下车道), obs2=施工区(挡上车道) => 迫使S形绕行
% ------------------------------------------------------------------------
% 每个障碍物是一个 Nx2 矩阵, 每行一个顶点 [x y], 顺时针, 首尾重复
obs{1} = [16, 0; 20.7, 0; 20.7, 2.2; 16, 2.2; 16, 0];   % 抛锚车(4.7x2.2), 贴下边界
obs{2} = [30, 3.0; 34, 3.0; 34, 8; 30, 8; 30, 3.0];     % 施工区, 贴上边界

map.obs = obs;
map.nOb = numel(obs);

%% ------------------------------------------------------------------------
%  4. 顶点表示 -> H-representation (A*y <= b), 供OBCA使用
% ------------------------------------------------------------------------
map.Aineq = cell(map.nOb,1);
map.bineq = cell(map.nOb,1);
for i = 1:map.nOb
    [A, b] = poly2hrep(obs{i});
    map.Aineq{i} = A;
    map.bineq{i} = b;
end

%% ------------------------------------------------------------------------
%  5. 生成占据栅格地图, 供 Hybrid A* 使用
% ------------------------------------------------------------------------
res = 5;                        % 分辨率 5 cells/m
occMap = binaryOccupancyMap(map.XYbounds(2), map.XYbounds(4), res);
% 用细网格逐点判断是否落在任一障碍物内, 设为占据
[xg, yg] = meshgrid(linspace(0,map.XYbounds(2),map.XYbounds(2)*res), ...
                    linspace(0,map.XYbounds(4),map.XYbounds(4)*res));
occ = false(size(xg));
for i = 1:map.nOb
    v = obs{i};
    in = inpolygon(xg, yg, v(:,1), v(:,2));
    occ = occ | in;
end
setOccupancy(occMap, [xg(occ), yg(occ)], 1);
map.occMap = occMap;

%% ------------------------------------------------------------------------
%  6. 可视化 (英文, 无网格, Zhang风格)
% ------------------------------------------------------------------------
figure('Color','w','Position',[100 100 950 260]); hold on; axis equal;
axis(map.XYbounds); box on;
set(gca,'FontName','Times New Roman','FontSize',12,'Layer','top');
xlabel('X [m]','FontName','Times New Roman','FontSize',13);
ylabel('Y [m]','FontName','Times New Roman','FontSize',13);
title('Simple Obstacle-Avoidance Scenario','FontName','Times New Roman','FontSize',14);

% 障碍物 (黑色填充, Zhang风格)
hObs = [];
for i = 1:map.nOb
    v = obs{i};
    hObs = fill(v(:,1), v(:,2), [0.15 0.15 0.15], 'EdgeColor','k','LineWidth',1.2);
end

% 起点/终点车身轮廓 + 参考点(后轴中心)
hStart = draw_vehicle(map.start, veh, [0 0.5 0]);   % 绿
hGoal  = draw_vehicle(map.goal,  veh, [0.85 0 0]);  % 红
hSP = plot(map.start(1),map.start(2),'o','Color',[0 0.5 0], ...
           'MarkerFaceColor',[0 0.5 0],'MarkerSize',6);
     plot(map.goal(1), map.goal(2),'o','Color',[0.85 0 0], ...
           'MarkerFaceColor',[0.85 0 0],'MarkerSize',6);
text(map.start(1),map.start(2)-1.3,'Start','Color',[0 0.5 0], ...
     'FontName','Times New Roman','FontWeight','bold','FontSize',11);
text(map.goal(1), map.goal(2)-1.3,'Goal','Color',[0.85 0 0], ...
     'FontName','Times New Roman','FontWeight','bold','FontSize',11);

% 显式指定图例, 避免颜色错乱
legend([hObs, hStart, hGoal, hSP], ...
       {'Obstacle','Start pose','Goal pose','Rear-axle ref. point'}, ...
       'FontName','Times New Roman','FontSize',10, ...
       'Location','northoutside','Orientation','horizontal','Box','off');

%% ------------------------------------------------------------------------
%  7. 打包数据接口 (后续模块与指标计算都从这个结构体取数据)
% ------------------------------------------------------------------------
scenario.veh = veh;
scenario.map = map;
save('scenario_simple.mat','scenario');
fprintf('地图建模完成: %d个障碍物, 已保存 scenario_simple.mat\n', map.nOb);


%% ========================================================================
%  局部函数
% ========================================================================
function [A, b] = poly2hrep(V)
% 凸多边形顶点(顺时针,首尾重复) -> A*y<=b (内部)
    V = V(1:end-1, :);              % 去掉重复的末点
    n = size(V,1);
    c = mean(V,1);                  % 质心(内部点)
    A = zeros(n,2); b = zeros(n,1);
    for k = 1:n
        p1 = V(k,:);
        p2 = V(mod(k,n)+1,:);
        e  = p2 - p1;               % 边向量
        nrm = [e(2), -e(1)];        % 法向量
        nrm = nrm / norm(nrm);
        if nrm*(c-p1)' > 0          % 保证法向指向外, 内部满足 <=
            nrm = -nrm;
        end
        A(k,:) = nrm;
        b(k)   = nrm*p1';
    end
end

function h = draw_vehicle(pose, veh, color)
% 根据后轴中心位姿[x y theta]画车身矩形, 返回handle
    x=pose(1); y=pose(2); th=pose(3);
    xu=veh.ego(1); yu=veh.ego(2); xl=veh.ego(3); yl=veh.ego(4);
    corners = [ xu,  yu;  xu, -yl; -xl, -yl; -xl, yu; xu, yu];
    R = [cos(th) -sin(th); sin(th) cos(th)];
    world = (R*corners')' + [x, y];
    h = plot(world(:,1), world(:,2), 'Color', color, 'LineWidth',1.8);
end