if ~exist('sim_phi', 'var')
    error('請先執行 Simulink，並確認 To Workspace 變數名稱為 sim_phi, sim_theta, sim_psi 等');
end

% 設定區
slow_factor = 0;  
step_size   = 5; 
d           = 0.2;

% 準備數據
N = length(sim_phi);
% 如果沒有位置數據，預設為原點
if ~exist('sim_x', 'var'), sim_x = zeros(N,1); end
if ~exist('sim_y', 'var'), sim_y = zeros(N,1); end
if ~exist('sim_z', 'var'), sim_z = zeros(N,1); end

% 計算固定視窗範圍
padding = 1.5; % 邊界留白
x_min = min(sim_x) - padding; x_max = max(sim_x) + padding;
y_min = min(sim_y) - padding; y_max = max(sim_y) + padding;
z_min = min(sim_z) - padding; z_max = max(sim_z) + padding;

if x_max - x_min < 2, x_min = -1; x_max = 1; end
if y_max - y_min < 2, y_min = -1; y_max = 1; end
if z_max - z_min < 2, z_min = -1; z_max = 1; end

% 初始化繪圖
figure(1); clf;
set(gcf, 'Color', 'w'); % 背景設為白色
grid on; axis equal;
xlabel('X (East)'); ylabel('Y (North)'); zlabel('Z (Up)');
view(3); % 3D 視角

% 設定固定的座標軸範圍
xlim([x_min x_max]);
ylim([y_min y_max]);
zlim([z_min z_max]);
hold on;

% 畫地板
patch([x_min x_max x_max x_min], [y_min y_min y_max y_max], [0 0 0 0], ...
      [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% 定義無人機外型
body_points = [ d,  d, 0;   % 1. 左前
               d, -d, 0;   % 2. 右前
              -d, -d, 0;   % 3. 右後
              -d,  d, 0;   % 4. 左後
               0,  0, 0;   % 5. 中心
               d,  0, 0];  % 6. 機頭指示

% 初始化繪圖物件
h_traj  = plot3(sim_x(1), sim_y(1), sim_z(1), 'b:', 'LineWidth', 1); % 軌跡線
h_drone = plot3(0,0,0, 'k-', 'LineWidth', 2); % 機身
h_head  = plot3(0,0,0, 'r-', 'LineWidth', 2); % 機頭
h_shadow= plot3(0,0,0, 'Color', [0.5 0.5 0.5], 'LineWidth', 1); % 地面投影

title('無人機飛行模擬 (按 Ctrl+C 可停止)');

% 動畫迴圈
for i = 1:step_size:N
    
    % 取得當前狀態
    phi = sim_phi(i); theta = sim_theta(i); psi = sim_psi(i);
    x = sim_x(i); y = sim_y(i); z = sim_z(i);
    
    % 旋轉矩陣
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R  = Rz * Ry * Rx;
    
    % 轉換機身點到世界座標
    world_points = (R * body_points')';
    world_points(:,1) = world_points(:,1) + x;
    world_points(:,2) = world_points(:,2) + y;
    world_points(:,3) = world_points(:,3) + z;
    

    % 畫 X 型
    set(h_drone, 'XData', [world_points(1,1) world_points(3,1) NaN world_points(2,1) world_points(4,1)], ...
                 'YData', [world_points(1,2) world_points(3,2) NaN world_points(2,2) world_points(4,2)], ...
                 'ZData', [world_points(1,3) world_points(3,3) NaN world_points(2,3) world_points(4,3)]);
    
    % 畫機頭
    set(h_head, 'XData', [world_points(5,1) world_points(6,1)], ...
                'YData', [world_points(5,2) world_points(6,2)], ...
                'ZData', [world_points(5,3) world_points(6,3)]);
    
    % 畫軌跡
    set(h_traj, 'XData', sim_x(1:step_size:i), ...
                'YData', sim_y(1:step_size:i), ...
                'ZData', sim_z(1:step_size:i));
            
    % 畫地面投影
    set(h_shadow, 'XData', [world_points(1,1) world_points(3,1) NaN world_points(2,1) world_points(4,1)], ...
                  'YData', [world_points(1,2) world_points(3,2) NaN world_points(2,2) world_points(4,2)], ...
                  'ZData', zeros(1,5)); % Z 固定為 0

    % 更新標題與時間
    title(sprintf('Time: %.2f s\nPos: [%.1f, %.1f, %.1f]\nAtt: R%.1f P%.1f Y%.1f', ...
          i*0.01, x, y, z, rad2deg(phi), rad2deg(theta), rad2deg(psi)));
    
    % 繪製與延遲
    drawnow; 
    pause(slow_factor);
end