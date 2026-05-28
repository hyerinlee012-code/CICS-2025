%% ================================
% 1. 맵 생성 
% ================================

map = mapMaze(5,Mapsize=[50,50],MapResolution=1);
Dynamicmap = mapMaze(5,Mapsize=[50,50],MapResolution=1);
%% 장애물 없는 맵 만들기
figure
show(map)
title ('Binary Occupancy Map without Obstacles')
hold off;
%% 정적 장애물맵
Obstaclemap = mapMaze(5,Mapsize=[50,50],MapResolution=1);
rng(1)
for i = 1:50
    x = rand*50; y = rand*50;
    setOccupancy(Obstaclemap,[x y],1)
end
% map plot
figure
show(Obstaclemap)
title('Binary Occupancy Map with Obstacles')
hold off

%% 6. 동적 장애물 업데이트 함수
function dynamicObstacles = updateDynamicObstacles(dynamicObstacles, dt)
    for i = 1:length(dynamicObstacles)
        obs = dynamicObstacles{i};
        
        % 위치 업데이트
        newPos = obs.pos + obs.velocity * dt;
        
        % 경계 체크 및 방향 반전
        if newPos(1) <= obs.bounds(1) || newPos(1) >= obs.bounds(2)
            obs.velocity(1) = -obs.velocity(1);
            newPos(1) = max(obs.bounds(1), min(obs.bounds(2), newPos(1)));
        end
        
        if newPos(2) <= obs.bounds(3) || newPos(2) >= obs.bounds(4)
            obs.velocity(2) = -obs.velocity(2);
            newPos(2) = max(obs.bounds(3), min(obs.bounds(4), newPos(2)));
        end
        
        obs.pos = newPos;
        dynamicObstacles{i} = obs;
    end
end

%% ================================
% 동적 장애물 환경에서 Hybrid A* vs RRT 비교
% ================================


% %% 1. 기본 맵 생성 (정적 장애물 포함)
BaseMap = binaryOccupancyMap(50, 50, 1);

%% 2. 동적 장애물 초기 설정
% 동적 장애물 개수
numObstacles = 3;

% 각 장애물의 초기 위치와 속도
obstacleData = [
    15, 25, 0.5, 0;      % [x, y, vx, vy] - 수평 이동
    25, 15, 0, 0.4;      % 수직 이동
    35, 35, -0.3, -0.3   % 대각선 이동
];

% 이동 범위 [x_min, x_max, y_min, y_max]
obstacleBounds = [
    5, 45, 20, 30;
    20, 30, 5, 45;
    10, 45, 10, 45
];

% 장애물 크기
obstacleSize = 1.5;

%% 3. 차량 설정
startPose = [5, 5, pi/4];
goalPose = [45, 45, pi/4];

%% 4. 동적 장애물 업데이트 함수
function [obstacleData] = updateObstacles(obstacleData, obstacleBounds, dt)
    for i = 1:size(obstacleData, 1)
        % 위치 업데이트
        obstacleData(i, 1) = obstacleData(i, 1) + obstacleData(i, 3) * dt;
        obstacleData(i, 2) = obstacleData(i, 2) + obstacleData(i, 4) * dt;

        % X 방향 경계 체크
        if obstacleData(i, 1) <= obstacleBounds(i, 1) || ...
           obstacleData(i, 1) >= obstacleBounds(i, 2)
            obstacleData(i, 3) = -obstacleData(i, 3);  % 속도 반전 반대 방향으로 이동하게 하기 위함
        end

        % Y 방향 경계 체크
        if obstacleData(i, 2) <= obstacleBounds(i, 3) || ...
           obstacleData(i, 2) >= obstacleBounds(i, 4)
            obstacleData(i, 4) = -obstacleData(i, 4);  % 속도 반전
        end
    end
end

%% 5. 맵에 동적 장애물 추가 함수
function [dynamicMap] = addDynamicObstacles(baseMap, obstacleData, obstacleSize)
    dynamicMap = copy(baseMap);

    for i = 1:size(obstacleData, 1)
        x = obstacleData(i, 1);
        y = obstacleData(i, 2);

        % 장애물 영역 설정
        [X, Y] = meshgrid(x-obstacleSize:0.2:x+obstacleSize, ...
                          y-obstacleSize:0.2:y+obstacleSize);
        coords = [X(:), Y(:)];
        setOccupancy(dynamicMap, coords, 1);
    end
end

a = 0;
b = 0;
%% 6. Hybrid A* 경로 계획 함수
function [refpath_d, planTime_d] = planHybridAStar(dynamicMap, startPose, goalPose)
global a
    % occupancyMap 변환
    occMatrix_d = occupancyMatrix(dynamicMap);
    omap_d = occupancyMap(occMatrix_d, dynamicMap.Resolution);

    % Validator 설정
    stateValidator_d = validatorOccupancyMap;
    stateValidator_d.Map = omap_d;
    stateValidator_d.ValidationDistance = 0.1;

    % Hybrid A* Planner
    planner_d = plannerHybridAStar(stateValidator_d, ...
        'MinTurningRadius', 4, ...
        'MotionPrimitiveLength', 2);  % MinTurningRadius : 차량 최소 회전 반경 

    %MotionPrimitiveLength (모션 프리미티브 길이) :

    % 시작/목표 위치 확인
    if getOccupancy(omap_d, startPose(1:2)) > 0
        setOccupancy(omap_d, startPose(1:2), 0);
    end
    if getOccupancy(omap_d, goalPose(1:2)) > 0
        setOccupancy(omap_d, goalPose(1:2), 0);
    end

    % 경로 계획
    tic;
    try
        refpath_d = plan(planner_d, startPose, goalPose);
        planTime_d = toc;
        fprintf('Hybrid A* 성공! 시간: %.3f초\n', planTime_d);
        a = a + planTime_d;
    catch ME
        refpath_d = [];
        planTime_d = toc;
        fprintf('Hybrid A* 실패: %s\n', ME.message);
    end
    disp(a)
end
%% 7. RRT 경로 계획 함수
function [pathObj_RRT_d, planTime_RRT_d] = planRRT_d(dynamicMap, startPose, goalPose)

global b
    % occupancyMap 변환
    occMatrix_RRT_d = occupancyMatrix(dynamicMap);
    omap_RRT_d = occupancyMap(occMatrix_RRT_d, dynamicMap.Resolution);

    % State Space 설정
    ss_d = stateSpaceSE2;
    ss_d.StateBounds = [omap_RRT_d.XWorldLimits; omap_RRT_d.YWorldLimits; [-pi pi]];

    % Validator 설정
    sv_d = validatorOccupancyMap(ss_d);
    sv_d.Map = omap_RRT_d;
    sv_d.ValidationDistance = 0.1;

    % RRT Planner 설정
    planner_d = plannerRRT(ss_d, sv_d);
    planner_d.MaxConnectionDistance = 1;
    planner_d.MaxIterations = 10000;
    %planner_d.GoalReachedFcn = @(~,q) norm(q(1:2) - goalPose(1:2)) < 1.0;

    % 시작/목표 위치 확인
    if getOccupancy(omap_RRT_d, startPose(1:2)) > 0
        setOccupancy(omap_RRT_d, startPose(1:2), 0);
    end
    if getOccupancy(omap_RRT_d, goalPose(1:2)) > 0
        setOccupancy(omap_RRT_d, goalPose(1:2), 0);
    end
  
    % 경로 계획 (최대 10번 시도)
  pathObj_RRT_d = []; 
  for attempt = 1:3 
      tic; 
      try 
          [pathObj_RRT_d, solnInfo_RRT_d] = plan(planner_d, startPose, goalPose); 
          planTime_RRT_d = toc; 
          b = b + planTime_RRT_d;
          if solnInfo_RRT_d.IsPathFound 
              fprintf('RRT 성공! 시도: %d, 시간: %.3f초\n', attempt, planTime_RRT_d);
              disp(b)
              return; 
          end 
      catch 
          planTime_RRT_d = toc; 
      end 
  end 
  fprintf('RRT 실패 (3번 시도 후)\n'); 
end

%% 8. 실시간 시뮬레이션
totalSteps = 100 ; %80;         % 총 시뮬레이션 스텝
dt = 0.5;                % 시간 간격
replanInterval = 10;     % 재계획 주기

% 초기 경로 계획
fprintf('\n=== 초기 경로 계획 ===\n');
currentMap = addDynamicObstacles(BaseMap, obstacleData, obstacleSize);

[refpath_hybrid, ~] = planHybridAStar(currentMap, startPose, goalPose);
[pathObj_rrt, ~] = planRRT_d(currentMap, startPose, goalPose);

% 초기 경로 상태
if ~isempty(refpath_hybrid)
    pathStates_hybrid = refpath_hybrid.States;
    pathIdx_hybrid = 1;
    vehiclePos_hybrid = startPose;
else
    pathStates_hybrid = [];
    pathIdx_hybrid = 1;
    vehiclePos_hybrid = startPose;
end

if ~isempty(pathObj_rrt)
    pathStates_rrt = pathObj_rrt.States;
    pathIdx_rrt = 1;
    vehiclePos_rrt = startPose;
else
    pathStates_rrt = [];
    pathIdx_rrt = 1;
    vehiclePos_rrt = startPose;
end

% 시각화 준비
figure('Position', [50, 50, 1400, 600]);

%% 9. 시뮬레이션 루프
for step = 1:totalSteps
    % 동적 장애물 업데이트
    obstacleData = updateObstacles(obstacleData, obstacleBounds, dt);

    % 주기적 재계획
    if mod(step, replanInterval) == 0
        fprintf('\n=== Step %d: 재계획 ===\n', step);

        % 맵 업데이트
        currentMap = addDynamicObstacles(BaseMap, obstacleData, obstacleSize);

        % Hybrid A* 재계획
        [refpath_hybrid, ~] = planHybridAStar(currentMap, vehiclePos_hybrid, goalPose);
        if ~isempty(refpath_hybrid)
            pathStates_hybrid = refpath_hybrid.States;
            pathIdx_hybrid = 1;
        end

        % RRT 재계획
        [pathObj_rrt, ~] = planRRT_d(currentMap, vehiclePos_rrt, goalPose);
        if ~isempty(pathObj_rrt)
            pathStates_rrt = pathObj_rrt.States;
            pathIdx_rrt = 1;
        end
    end

    % 차량 이동 (경로 추종)
    if ~isempty(pathStates_hybrid) && pathIdx_hybrid <= size(pathStates_hybrid, 1)
        vehiclePos_hybrid = pathStates_hybrid(pathIdx_hybrid, :);
        pathIdx_hybrid = pathIdx_hybrid + 1;
    end

    if ~isempty(pathStates_rrt) && pathIdx_rrt <= size(pathStates_rrt, 1)
        vehiclePos_rrt = pathStates_rrt(pathIdx_rrt, :);
        pathIdx_rrt = pathIdx_rrt + 1;
    end

    % 시각화
    % Hybrid A* 서브플롯
    subplot(1, 2, 1);
    show(currentMap);
    hold on;

    if ~isempty(pathStates_hybrid)
        plot(pathStates_hybrid(:, 1), pathStates_hybrid(:, 2), 'r-', 'LineWidth', 2.5);
    end
    plot(vehiclePos_hybrid(1), vehiclePos_hybrid(2), 'go', ...
        'MarkerSize', 18, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(goalPose(1), goalPose(2), 'b^', ...
        'MarkerSize', 18, 'MarkerFaceColor', 'b', 'LineWidth', 2);

    % 동적 장애물 표시
    for i = 1:size(obstacleData, 1)
        plot(obstacleData(i, 1), obstacleData(i, 2), 'rx', ...
            'MarkerSize', 25, 'LineWidth', 4);
    end

    dist_hybrid = norm(vehiclePos_hybrid(1:2) - goalPose(1:2));
    title(sprintf('Hybrid A* (Step %d) - 목표까지: %.1fm', step, dist_hybrid), 'FontSize', 12);
    legend('', '계획 경로', '차량', '목표', '동적 장애물', 'Location', 'best');
    hold off;

    % RRT 서브플롯
    subplot(1, 2, 2);
    show(currentMap);
    hold on;

    if ~isempty(pathStates_rrt)
        plot(pathStates_rrt(:, 1), pathStates_rrt(:, 2), 'b-', 'LineWidth', 2.5);
    end
    plot(vehiclePos_rrt(1), vehiclePos_rrt(2), 'mo', ...
        'MarkerSize', 18, 'MarkerFaceColor', 'm', 'LineWidth', 2);
    plot(goalPose(1), goalPose(2), 'b^', ...
        'MarkerSize', 18, 'MarkerFaceColor', 'b', 'LineWidth', 2);

    % 동적 장애물 표시
    for i = 1:size(obstacleData, 1)
        plot(obstacleData(i, 1), obstacleData(i, 2), 'rx', ...
            'MarkerSize', 25, 'LineWidth', 4);
    end

    dist_rrt = norm(vehiclePos_rrt(1:2) - goalPose(1:2));
    title(sprintf('RRT (Step %d) - 목표까지: %.1fm', step, dist_rrt), 'FontSize', 12);
    legend('', '계획 경로', '차량', '목표', '동적 장애물', 'Location', 'best');
    hold off;

    drawnow;
    pause(0.15);

    % 목표 도달 확인
    if dist_hybrid < 2 && dist_rrt < 2
        fprintf('\n두 알고리즘 모두 목표 도달!\n');
        break;
    end
end

fprintf('\n=== 시뮬레이션 완료 ===\n')

% ---------- 경로 길이 계산 (안전한 버전) ----------
% Hybrid A* 경로 길이
if ~isempty(pathStates_hybrid) && size(pathStates_hybrid,1) >= 2
    hx = diff(pathStates_hybrid(:,1));
    hy = diff(pathStates_hybrid(:,2));
    segmentDistances_hybrid = hypot(hx, hy);
    totalDistance_hybrid = sum(segmentDistances_hybrid);
else
    totalDistance_hybrid = 0;
end

fprintf("Hybrid A*의 동적 맵 총 경로 길이 : %.2f m\n", totalDistance_hybrid);

% RRT 경로 길이
if ~isempty(pathStates_rrt) && size(pathStates_rrt,1) >= 2
    rx = diff(pathStates_rrt(:,1));
    ry = diff(pathStates_rrt(:,2));
    segmentDistances_rrt = hypot(rx, ry);
    totalDistance_rrt = sum(segmentDistances_rrt);
else
    totalDistance_rrt = 0;
end

fprintf("RRT의 동적 맵 총 경로 길이 : %.2f m\n", totalDistance_rrt);
% ----------------------------------------------------


% %pathStates_curve = refpath_curve.States;   % [x y theta]
% 
% % 거리 계산 (x, y만 사용)
% dx = diff(vehiclePos_hybrid(:,1));
% dy = diff(vehiclePos_hybrid(:,2));
% segmentDistances_dynamic = hypot(dx, dy);
% totalDistance_dynamic = sum(segmentDistances_dynamic);
% 
% fprintf("Hybrid A*의 동적 맵 총 경로 길이 : %.2f m\n", totalDistance_curve);
% 
% %pathStates_RRT_curve = pathObj_curve.States;   % [x y theta]
% 
%  % 거리 계산 (x, y만 사용)
%  disp (pathStates_rrt(:,2))
%  dy_RRT_dynamic = diff(pathStates_rrt(:,2));
%  segmentDistances_rrt_dynamic= hypot(dx_RRT_dynamic, dy_RRT_dynamic);
%  totalDistance_RRT_dynamic = sum(segmentDistances_rrt_dynamic);
% 
% fprintf("RRT의 동적 맵 총 경로 길이 : %.2f m\n", totalDistance_RRT_dynamic); 

%% ================================
%  커브 맵 (S자 장애물 맵) 생성
% ================================

mapWidth = 50;     % 50 m
mapHeight = 50;    % 50 m
resolution = 1;    % 1 grid = 1 m
map_curve = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% ================================
%  곡선 궤적 정의 (2개의 베지어 곡선)
% ================================
t = linspace(0,1,200); % 곡선의 세밀함 조절

% 윗쪽 곡선
P0 = [0 15]; 
P1 = [25 40]; 
P2 = [50 15];

% 아래쪽 곡선
P3 = [0 5];  
P4 = [25 35]; 
P5 = [50 5];

% 2차 베지어 곡선 공식 (Quadratic Bezier)
x_upper = (1-t).^2 * P0(1) + 2*(1-t).*t*P1(1) + t.^2*P2(1);
y_upper = (1-t).^2 * P0(2) + 2*(1-t).*t*P1(2) + t.^2*P2(2);

x_lower = (1-t).^2 * P3(1) + 2*(1-t).*t*P4(1) + t.^2*P5(1);
y_lower = (1-t).^2 * P3(2) + 2*(1-t).*t*P4(2) + t.^2*P5(2);

% ================================
%  곡선 폭 설정 (장애물 두께)
% ================================
curveWidth = 0.3; 

% ================================
%  곡선 장애물 생성
% ================================
for i = 1:length(x_upper)
    [X, Y] = meshgrid(x_upper(i)-curveWidth/2 : 0.1 : x_upper(i)+curveWidth/2, ...
                      y_upper(i)-curveWidth/2 : 0.1 : y_upper(i)+curveWidth/2);
    setOccupancy(map_curve, [X(:), Y(:)], 1);

    [X1, Y1] = meshgrid(x_lower(i)-curveWidth/2 : 0.1 : x_lower(i)+curveWidth/2, ...
                        y_lower(i)-curveWidth/2 : 0.1 : y_lower(i)+curveWidth/2);
    setOccupancy(map_curve, [X1(:), Y1(:)], 1);
end

% ================================
%  시각화
% ================================
figure;
show(map_curve);
title('Curved Map');
%axis equal;

%% ================================
% 2. Hybrid A* (차량 동역학 고려)
% ================================

 occMatrix = occupancyMatrix(map);  % 0과 1로 된 행렬
 occMatrix_o = occupancyMatrix(Obstaclemap);
 occMatrix_park = occupancyMatrix(map_park);
 occMatrix_curve = occupancyMatrix(map_curve);
%% occupancyMap으로 변환 (해상도 동일하게 설정)

omap = occupancyMap(occMatrix,map.Resolution);
omap_o = occupancyMap(occMatrix_o,Obstaclemap.Resolution);
omap_park = occupancyMap(occMatrix_park,map_park.Resolution);
omap_curve = occupancyMap(occMatrix_curve,map_curve.Resolution);
%% ===========Hybrid A* Planner 생성==========

stateValidator = validatorOccupancyMap;
stateValidator.Map = omap;
stateValidator.ValidationDistance = 0.1;

stateValidator_o = validatorOccupancyMap;
stateValidator_o.Map = omap_o;
stateValidator_o.ValidationDistance = 0.1;

stateValidator_park = validatorOccupancyMap;
stateValidator_park.Map = omap_park;
stateValidator_park.ValidationDistance = 0.1;

stateValidator_curve = validatorOccupancyMap;
stateValidator_curve.Map = omap_curve;
stateValidator_curve.ValidationDistance = 0.1;

% 차량 모델
vehicleDims = vehicleDimensions(1.8, 4.5   ); % 길이, 너비 (실제 자동차의 크기와 비슷하게 진행해줌) 

%경로계획세우기
plannerHybrid = plannerHybridAStar(stateValidator,'MinTurningRadius',4,'MotionPrimitiveLength',2);
plannerHybrid_o = plannerHybridAStar(stateValidator_o,'MinTurningRadius',4,'MotionPrimitiveLength',2);
plannerHybrid_curve = plannerHybridAStar(stateValidator_curve,'MinTurningRadius',4,'MotionPrimitiveLength',2);

startPose = [5 5 pi/4];   % x,y,heading(rad)
goalPose = [47 47 pi/4]; 

startPose_curve = [1 12 pi/4];   % x,y,heading(rad)
goalPose_curve = [46 12  pi/4];  % 빈 값으로 초기화

if getOccupancy(omap,goalPose(1:2)) > 0
    setOccupancy(omap,goalPose(1:2),0); % goal 지점을 빈공간으로 설정
end

if getOccupancy(omap_o,startPose(1:2)) > 0
    setOccupancy(omap_o,startPose(1:2),0); % goal 지점을 빈공간으로 설정
end

if getOccupancy(omap_park,startPose_park(1:2)) > 0
    setOccupancy(omap_park,startPose_park(1:2),0); % goal 지점을 빈공간으로 설정
end


% 경로 계산
tic;
refpath = plan(plannerHybrid,startPose,goalPose);
t1 = toc;       % 경과 시간 저장
fprintf('Hybrid A*의 기본 맵 걸린 시간: %.2f 초\n', t1);
tic;
refpath_o = plan(plannerHybrid_o,startPose,goalPose);
t2 = toc;       % 경과 시간 저장
fprintf('Hybrid A*의 장애물 맵 걸린 시간: %.2f 초\n', t2);
tic;
refpath_park = plan(plannerHybrid_park,startPose_park,goalPose_park);
t3 = toc; 
fprintf('Hybrid A*의 주차장 맵 걸린 시간: %.2f 초\n', t3);
tic;
refpath_curve = plan(plannerHybrid_curve,startPose_curve,goalPose_curve);
t4 = toc; 
fprintf('Hybrid A*의 커브 맵 걸린 시간: %.2f 초\n', t4);
%% ======RRT planner 경로 계획=====
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss); 
sv.Map = omap;           % occupancy map
sv.ValidationDistance = 0.1;
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 1.5;  % 트리 확장 거리
ss.StateBounds = [omap.XWorldLimits; omap.YWorldLimits; [-pi pi]];

tic;
[pathObj, solnInfo] = plan(planner, startPose, goalPose);
t1_RRT = toc; 
fprintf('RRT의 기본 맵 걸린 시간: %.2f 초\n', t1_RRT);

omap = sv.Map;          % binaryOccupancyMap
pts = pathObj.States;   % [x y theta]
n = size(pts,1);

collidingLengths = 0;

for i = 1:n-1
    % 두 점이 모두 장애물에 해당하면
    isCollide1 = getOccupancy(omap, pts(i,1:2)) > 0.2;
    isCollide2 = getOccupancy(omap, pts(i+1,1:2)) > 0.2;
    
    if isCollide1 && isCollide2
        dx = pts(i+1,1) - pts(i,1);
        dy = pts(i+1,2) - pts(i,2);
        collidingLengths = collidingLengths + sqrt(dx^2 + dy^2);
    end
end

disp(['충돌 구간 총 길이 = ', num2str(collidingLengths), ' m']);



%정적 장애물 맵 
ss_o = stateSpaceSE2;
sv_o = validatorOccupancyMap(ss_o); 
sv_o.Map = omap_o;           % occupancy map
sv_o.ValidationDistance = 0.1;
planner_o = plannerRRT(ss_o,sv_o);
planner_o.MaxConnectionDistance = 1.5;  % 트리 확장 거리
ss_o.StateBounds = [omap_o.XWorldLimits; omap.YWorldLimits; [-pi pi]];

tic;
[pathObj_o, solnInfo_o] = plan(planner_o, startPose, goalPose);
t2_RRT = toc; 
fprintf('RRT의 장애물 맵 걸린 시간: %.2f 초\n', t2_RRT);

%커브 맵 
ss_curve = stateSpaceSE2;
sv_curve = validatorOccupancyMap(ss_curve); 
sv_curve.Map = omap_curve;           % occupancy map
sv_curve.ValidationDistance = 0.1;
planner_curve = plannerRRT(ss_curve,sv_curve);
planner_curve.MaxConnectionDistance = 2;  % 트리 확장 거리
ss_curve.StateBounds = [omap_curve.XWorldLimits; omap_curve.YWorldLimits; [-pi pi]];

tic;
[pathObj_curve, solnInfo_curve] = plan(planner_curve, startPose_curve, goalPose_curve);
t4_RRT = toc; 
fprintf('RRT의 커브 맵 걸린 시간: %.2f 초\n', t4_RRT);


%% =====시각화=====
figure
show(omap)
hold on
plot(refpath.States(:,1),refpath.States(:,2),'r-','LineWidth',2)
%quiver(refpath.States(:,1),refpath.States(:,2),cos(refpath.States(:,3)),sin(refpath.States(:,3)))
plot(startPose(1),startPose(2),'go','MarkerSize',10,'MarkerFaceColor','g')
plot(goalPose(1),goalPose(2),'bo','MarkerSize',10,'MarkerFaceColor','b')
title('Binary Occupancy Map without Obstacles')
plot(pathObj.States(:,1), pathObj.States(:,2), 'b-', 'LineWidth', 2)
legend('Hybrid A*','Start','Goal','RRT')
hold off

figure
show(omap_o)
hold on
plot(refpath_o.States(:,1),refpath_o.States(:,2),'r-','LineWidth',2)
%quiver(refpath.States(:,1),refpath.States(:,2),cos(refpath.States(:,3)),sin(refpath.States(:,3)))
plot(startPose(1),startPose(2),'go','MarkerSize',10,'MarkerFaceColor','g')
plot(goalPose(1),goalPose(2),'bo','MarkerSize',10,'MarkerFaceColor','b')
title('Binary Occupancy Map with Obstacles')
plot(pathObj_o.States(:,1), pathObj_o.States(:,2), 'b-', 'LineWidth', 2)
legend('Hybrid A*','Start','Goal','RRT')


figure
show(omap_curve)
hold on
plot(refpath_curve.States(:,1),refpath_curve.States(:,2),'r-','LineWidth',2)
%quiver(refpath.States(:,1),refpath.States(:,2),cos(refpath.States(:,3)),sin(refpath.States(:,3)))
plot(startPose_curve(1),startPose_curve(2),'go','MarkerSize',10,'MarkerFaceColor','g')
plot(goalPose_curve(1),goalPose_curve(2),'bo','MarkerSize',10,'MarkerFaceColor','b')
title('Curved Map')
plot(pathObj_curve.States(:,1), pathObj_curve.States(:,2), 'b-', 'LineWidth', 2)
legend('Hybrid A*','Start','Goal','RRT')

%% =====거리 계산=====
%기본 맵 거리 계산 
pathStates = refpath.States;   % [x y theta]

% 거리 계산 (x, y만 사용)
dx = diff(pathStates(:,1));
dy = diff(pathStates(:,2));
segmentDistances = hypot(dx, dy);
totalDistance = sum(segmentDistances);

fprintf("Hybrid A*의 기본 맵 총 경로 길이 : %.2f m\n", totalDistance);

pathStates_RRT = pathObj.States;   % [x y theta]

% 거리 계산 (x, y만 사용)
dx = diff(pathStates_RRT(:,1));
dy = diff(pathStates_RRT(:,2));
segmentDistances_RRT = hypot(dx, dy);
totalDistance_RRT = sum(segmentDistances_RRT);

fprintf("RRT의 기본 맵 총 경로 길이 : %.2f m\n", totalDistance_RRT);


% 장애물 맵 거리 계산
pathStates_o = refpath_o.States;   % [x y theta]

% 거리 계산 (x, y만 사용)
dx = diff(pathStates_o(:,1));
dy = diff(pathStates_o(:,2));
segmentDistances_o = hypot(dx, dy);
totalDistance_o = sum(segmentDistances_o);

fprintf("Hybrid A*의 장애물 맵 총 경로 길이 : %.2f m\n", totalDistance_o);

 %RRT의 장애물 맵 거리 계산 
 pathStates_RRT_ = pathObj.States;   % [x y theta]
 
 % 거리 계산 (x, y만 사용)
 dx = diff(pathStates_RRT(:,1));
 dy = diff(pathStates_RRT(:,2));
 segmentDistances_rrt_o = hypot(dx, dy);
 totalDistance_RRT = sum(segmentDistances_rrt_o);

fprintf("RRT의 장애물 맵 총 경로 길이 : %.2f m\n", totalDistance);

% 장애물 맵 거리 계산
pathStates_park = refpath_park.States;   % [x y theta]

% 거리 계산 (x, y만 사용)
dx = diff(pathStates_park(:,1));
dy = diff(pathStates_park(:,2));
segmentDistances_park = hypot(dx, dy);
totalDistance_park = sum(segmentDistances_park);

fprintf("Hybrid A*의 주차장 맵 총 경로 길이 : %.2f m\n", totalDistance_park);

pathStates_curve = refpath_curve.States;   % [x y theta]

% 거리 계산 (x, y만 사용)
dx = diff(pathStates_curve(:,1));
dy = diff(pathStates_curve(:,2));
segmentDistances_curve = hypot(dx, dy);
totalDistance_curve = sum(segmentDistances_curve);

fprintf("Hybrid A*의 커브 맵 총 경로 길이 : %.2f m\n", totalDistance_curve);

pathStates_RRT_curve = pathObj_curve.States;   % [x y theta]
 
 % 거리 계산 (x, y만 사용)
 dx = diff(pathStates_RRT_curve(:,1));
 dy = diff(pathStates_RRT_curve(:,2));
 segmentDistances_rrt_curve = hypot(dx, dy);
 totalDistance_RRT_curve = sum(segmentDistances_rrt_curve);

fprintf("RRT의 커브 맵 총 경로 길이 : %.2f m\n", totalDistance_RRT_curve); 