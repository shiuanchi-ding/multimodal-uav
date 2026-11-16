clear; clc; close all;

COM_PORT = "COM8";  
BAUD     = 9600;  
MAX_POINTS = 3000;      % 收集點數上限

%% 初始化 Serial
fprintf('正在連線到 %s ...\n', COM_PORT);
try
    s = serialport(COM_PORT, BAUD);
    configureTerminator(s, "CR/LF");
    flush(s);
    disp('連線成功！');
catch ME
    error(['無法連線！請確認：' newline ...
           '1. COM Port 號碼是否正確 (要選傳出/Outgoing)' newline ...
           '2. 藍牙是否已配對' newline ...
           '3. 是否有其他程式佔用此 Port']);
end

disp('=== 磁力計校正開始 ===');
disp('請拿起傳感器/機器人，在空中向各個角度旋轉 (畫 8 字)。');
disp('目標是將紅點填滿成一個球體。');
disp('按任意鍵開始接收數據...');
pause; 

% 建立圖形視窗，並把 handle 存入 hFig
hFig = figure('Name', 'Magnetometer Calibration Data', 'Color', 'w');
hAx = axes(hFig);
hScatter = scatter3(hAx, 0, 0, 0, 'filled', 'MarkerFaceColor', 'r');
axis equal; grid on; xlabel('Mx'); ylabel('My'); zlabel('Mz');
title('請旋轉傳感器填滿球體');
view(3);

%% 收集數據
raw_data = [];
count = 0;
s.Timeout = 0.5; % 縮短 Timeout 時間，讓程式反應快一點

disp('開始接收...若一直沒反應，請檢查藍牙連線。');

while count < MAX_POINTS && ishandle(hFig)
    
    % 1. 檢查緩衝區
    if s.NumBytesAvailable == 0
        pause(0.05);
        title(hAx, '等待訊號中... (目前無數據)');
        continue;
    end

    % 2. 安全讀取
    try
        line = readline(s);
    catch
        continue;
    end
    
    % 3. 檢查空值、缺失值、字串長度不足
    if isempty(line) || (isstring(line) && ismissing(line)) || strlength(line) < 5
        continue; 
    end

    % 4. 嘗試解碼
    try
        str_vals = split(strtrim(line), ',');
        vals = str2double(str_vals);
        
        % 檢查是否有 10 個數值且非 NaN
        if numel(vals) == 10 && ~any(isnan(vals))
            mx = vals(8); my = vals(9); mz = vals(10);
            
            % 過濾全 0
            if (mx==0 && my==0 && mz==0), continue; end

            raw_data = [raw_data; mx, my, mz]; %#ok<AGROW>
            count = count + 1;
            
            % 繪圖更新
            if mod(count, 20) == 0
                set(hScatter, 'XData', raw_data(:,1), ...
                              'YData', raw_data(:,2), ...
                              'ZData', raw_data(:,3));
                title(hAx, sprintf('收集進度: %d / %d', count, MAX_POINTS));
                drawnow limitrate;
                if s.NumBytesAvailable > 500, flush(s); end
            end
        end
    catch
        continue; % 解碼失敗就跳過
    end
end

clear s; % 釋放 COM Port
if ~ishandle(hFig)
    disp('使用者關閉視窗，停止收集。');
else
    disp('收集完成！開始計算參數...');
end

%% 計算校正參數
if isempty(raw_data)
    error('沒有收集到任何數據，無法計算。');
end

m_min = min(raw_data);
m_max = max(raw_data);

% Hard-iron Bias
mag_bias = (m_max + m_min)' / 2;

% Soft-iron Scale
chord_x = (m_max(1) - m_min(1)) / 2;
chord_y = (m_max(2) - m_min(2)) / 2;
chord_z = (m_max(3) - m_min(3)) / 2;

avg_rad = (chord_x + chord_y + chord_z) / 3;

scale_x = avg_rad / chord_x;
scale_y = avg_rad / chord_y;
scale_z = avg_rad / chord_z;

mag_scale = diag([scale_x, scale_y, scale_z]);

calib_data = (raw_data - mag_bias') * mag_scale;

% 畫對比圖
figure('Name', '校正結果對比', 'Color', 'w');
subplot(1,2,1);
scatter3(raw_data(:,1), raw_data(:,2), raw_data(:,3), 10, 'r');
axis equal; grid on; title('校正前 (原始)');
subplot(1,2,2);
scatter3(calib_data(:,1), calib_data(:,2), calib_data(:,3), 10, 'b');
[sx, sy, sz] = sphere(20);
hold on; mesh(sx*avg_rad, sy*avg_rad, sz*avg_rad, 'FaceAlpha', 0.1, 'EdgeColor', 'k');
axis equal; grid on; title('校正後 (理想球體)');

% 輸出參數
fprintf('\n\n=== 校正成功！請複製以下兩行到你的 IMU 程式中 ===\n');
fprintf('MAG_BIAS  = [%.5f; %.5f; %.5f];\n', mag_bias(1), mag_bias(2), mag_bias(3));
fprintf('MAG_SCALE = [%.5f, 0, 0; 0, %.5f, 0; 0, 0, %.5f];\n', ...
        mag_scale(1,1), mag_scale(2,2), mag_scale(3,3));
fprintf('=====================================================\n');