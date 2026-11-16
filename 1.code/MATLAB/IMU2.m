clear; clc; close all;

COM_PORT = "COM8";
BAUD     = 9600;
ACC_LSB_PER_G   = 16384.0;
GYR_LSB_PER_DPS = 16.4;

% Kalman Filter 參數
BIAS_LOCK_SEC   = 3.0;        % 前 3 秒靜止校正 Gyro Bias
Q_ANGLE   = 0.001;            % 角度狀態雜訊
Q_BIAS    = 0.003;            % Bias 漂移雜訊
R_ACC_DEG = 0.5;              % 加速度量測雜訊
R_MAG_DEG = 2.0;              % 磁力計量測雜訊

% 磁力計校正
MAG_BIAS  = [317.00000; 513.00000; 138.00000];        
MAG_SCALE = [0.83375, 0, 0; 0, 0.77585, 0; 0, 0, 1.95432];           
YAW_OFFSET_DEG = -4.5;           % 磁偏角補償

fprintf('正在連接 %s ... ', COM_PORT);
try
    s = serialport(COM_PORT, BAUD);
    configureTerminator(s, "CR/LF");
    flush(s);
    fprintf('成功！\n');
catch ME
    fprintf('失敗！\n錯誤訊息: %s\n', ME.message);
    return;
end

%% 初始化變數
% 狀態變數
biasLocked = false;
bx = 0; by = 0; bz = 0;
calib_gx = []; calib_gy = []; calib_gz = [];

% 時間與角度
t_prev = 0;
firstRun = true;
roll_zero = 0; pitch_zero = 0; yaw_zero = 0; % 用於 Tare (歸零)

% 初始化 Kalman Filter
KF_x = initKF(Q_ANGLE, Q_BIAS, R_ACC_DEG^2);
KF_y = initKF(Q_ANGLE, Q_BIAS, R_ACC_DEG^2);
KF_z = initKF(Q_ANGLE, Q_BIAS, R_MAG_DEG^2);

% 當前解算角度
cur_roll = 0; cur_pitch = 0; cur_yaw = 0;

%% 建立 3D 視窗
fig = figure('Name','Drone Attitude (BMX160)', 'Color','w', ...
    'KeyPressFcn', @(src,evt) setappdata(src,'key',evt.Key));
ax3d = axes(fig); 
grid(ax3d,'on'); axis(ax3d,'equal'); hold(ax3d,'on');
xlabel(ax3d,'X (Nose)'); ylabel(ax3d,'Y (Right)'); zlabel(ax3d,'Z (Down)'); 
view(ax3d, 35, 25);
xlim(ax3d,[-0.5 0.5]); ylim(ax3d,[-0.5 0.5]); zlim(ax3d,[-0.5 0.5]);

% 畫一個代表無人機的立方體
ht = hgtransform('Parent',ax3d);
[cubeV,cubeF,cubeC] = makeBodyCube(0.25, 0.25, 0.05); 
patch('Vertices',cubeV,'Faces',cubeF,'FaceVertexCData',cubeC, ...
      'FaceColor','flat','EdgeColor',[0.2 0.2 0.2], ...
      'Parent',ht,'FaceAlpha',0.9);
% 畫座標軸 (紅X, 綠Y, 藍Z)
quiver3(0,0,0,0.4,0,0,'r','LineWidth',2,'Parent',ht); 
quiver3(0,0,0,0,0.4,0,'g','LineWidth',2,'Parent',ht); 
quiver3(0,0,0,0,0,0.4,'b','LineWidth',2,'Parent',ht); 

light(ax3d,'Position',[1 1 -1]); 
lighting(ax3d, 'gouraud'); material(ax3d,'dull');

txt_info = text(ax3d, 0, 0, 0.6, 'Initializing...', ...
    'HorizontalAlignment','center', 'FontSize', 12, 'FontWeight','bold');

setappdata(fig,'key','');
fprintf('程式開始。請保持靜止 3 秒以校正 Gyro。\n按 "z" 可重設目前角度為 0。\n');

%% 主迴圈
while ishandle(fig)
    % Serial 防延遲讀取 讀取直到 Buffer 清空，只取最後一筆最新的數據
    if s.NumBytesAvailable == 0
        drawnow limitrate; continue; 
    end
    
    last_line = "";
    while s.NumBytesAvailable > 0
        temp_line = readline(s);
        if strlength(temp_line) > 5, last_line = temp_line; end
    end
    
    if strlength(last_line) < 5, continue; end
    
    % 資料解析
    % 預期 CSV 格式: t_ms, ax, ay, az, gx, gy, gz, mx, my, mz
    try
        vals = str2double(split(strtrim(last_line),','));
    catch
        continue;
    end
    
    if numel(vals) ~= 10 || any(isnan(vals)), continue; end
    
    % 時間處理
    t_now = vals(1) / 1000.0; 
    
    % 物理量轉換
    % 加速度轉成 g
    ax = vals(2) / ACC_LSB_PER_G;
    ay = vals(3) / ACC_LSB_PER_G;
    az = vals(4) / ACC_LSB_PER_G;
    
    % 角速度轉成 deg/s
    gx_val = vals(5) / GYR_LSB_PER_DPS;
    gy_val = vals(6) / GYR_LSB_PER_DPS;
    gz_val = vals(7) / GYR_LSB_PER_DPS;
    
    % 磁力計
    m_raw = [vals(8); vals(9); vals(10)];
    m_cal = MAG_SCALE * (m_raw - MAG_BIAS);
    m_norm = m_cal / max(1e-9, norm(m_cal)); % 正規化
    
    % 計算 dt
    if firstRun
        t_prev = t_now; firstRun = false; dt = 0;
    else
        dt = t_now - t_prev; t_prev = t_now;
        if dt > 0.1 || dt < 0, dt = 0; end % 異常過濾
    end
    
    % 靜止校正 Bias
    if ~biasLocked
        calib_gx(end+1) = gx_val;
        calib_gy(end+1) = gy_val;
        calib_gz(end+1) = gz_val;
        
        txt_info.String = sprintf('Calibrating... %.1fs', BIAS_LOCK_SEC - t_now);
        
        if t_now >= BIAS_LOCK_SEC
            bx = mean(calib_gx);
            by = mean(calib_gy);
            bz = mean(calib_gz);
            biasLocked = true;
            
            % 校正完畢，將初始角度設為目前的加速度角度
            roll_init  = rad2deg(atan2(ay, az));
            pitch_init = rad2deg(atan2(-ax, sqrt(ay^2 + az^2)));
            
            KF_x.angle = roll_init; KF_x.bias = bx;
            KF_y.angle = pitch_init; KF_y.bias = by;
            KF_z.angle = 0; KF_z.bias = bz;
            
            fprintf('Bias Locked: bx=%.2f, by=%.2f, bz=%.2f\n', bx, by, bz);
            
            % 釋放記憶體
            calib_gx = []; calib_gy = []; calib_gz = [];
        end
        drawnow limitrate; continue;
    end
    
    % AHRS
    
    % 1. 扣除 Gyro Bias
    gx_c = gx_val - bx;
    gy_c = gy_val - by;
    gz_c = gz_val - bz;
    
    % 2. 計算 Accel 角度
    roll_acc  = rad2deg(atan2(ay, az));
    pitch_acc = rad2deg(atan2(-ax, sqrt(ay*ay + az*az)));
    
    % 3. 判斷 Acc 是否可信
    a_mag = sqrt(ax^2 + ay^2 + az^2);
    useAcc = abs(a_mag - 1.0) < 0.2; 
    
    % 4. 計算 Mag Yaw 
    phi = deg2rad(KF_x.angle);
    the = deg2rad(KF_y.angle);
    mx = m_norm(1); my = m_norm(2); mz = m_norm(3);
    
    mxh = mx*cos(the) + mz*sin(the);
    myh = mx*sin(phi)*sin(the) + my*cos(phi) - mz*sin(phi)*cos(the);
    
    yaw_mag = rad2deg(atan2(-myh, mxh)) + YAW_OFFSET_DEG;
    useMag = (norm(m_norm) > 0.1); 

    % 5. Kalman Update
    if dt > 0
        KF_x = kfStep(KF_x, gx_c, roll_acc,  dt, useAcc);
        KF_y = kfStep(KF_y, gy_c, pitch_acc, dt, useAcc);
        KF_z = kfStep(KF_z, gz_c, yaw_mag,   dt, useMag);
    end
    
    cur_roll  = KF_x.angle;
    cur_pitch = KF_y.angle;
    cur_yaw   = KF_z.angle;

    % 顯示 
    key = getappdata(fig,'key');
    if strcmpi(key,'z')
        roll_zero = cur_roll; pitch_zero = cur_pitch; yaw_zero = cur_yaw;
        setappdata(fig,'key','');
        fprintf('Zero Set.\n');
    end
    
    % 顯示角度
    d_roll  = wrapTo180(cur_roll - roll_zero);
    d_pitch = wrapTo180(cur_pitch - pitch_zero);
    d_yaw   = wrapTo180(cur_yaw - yaw_zero);
    
    % 更新 3D
    Rx = makehgtform('xrotate', deg2rad(d_roll));
    Ry = makehgtform('yrotate', deg2rad(d_pitch));
    Rz = makehgtform('zrotate', deg2rad(d_yaw));
    set(ht, 'Matrix', Rz * Ry * Rx);
    
    txt_info.String = sprintf('Roll: %.1f  Pitch: %.1f  Yaw: %.1f\nAccNorm: %.2f  Freq: %.0fHz', ...
        d_roll, d_pitch, d_yaw, a_mag, 1/max(dt,0.001));
    
    drawnow limitrate;
end

clear s; 

%% Functions
function KF = initKF(q_angle, q_bias, r_meas)
    KF.angle = 0; KF.bias = 0;
    KF.P = [0.1, 0; 0, 0.1];
    KF.Q = [q_angle, 0; 0, q_bias];
    KF.R = r_meas;
end

function KF = kfStep(KF, gyro, meas, dt, useMeas)
    % 1. Predict
    rate = gyro - KF.bias;
    KF.angle = KF.angle + rate * dt;
    
    P = KF.P;
    P(1,1) = P(1,1) - dt*P(2,1) - dt*P(1,2) + dt*dt*P(2,2) + KF.Q(1,1);
    P(1,2) = P(1,2) - dt*P(2,2);
    P(2,1) = P(2,1) - dt*P(2,2);
    P(2,2) = P(2,2) + KF.Q(2,2);
    
    KF.angle = wrapTo180(KF.angle);

    % 2. Update
    if useMeas
        y = wrapTo180(meas - KF.angle); % Innovation
        S = P(1,1) + KF.R;
        K = [P(1,1)/S; P(2,1)/S];       % Kalman Gain
        
        KF.angle = KF.angle + K(1)*y;
        KF.bias  = KF.bias  + K(2)*y;
        
        % P = (I - KH)P
        P_old = P;
        P(1,1) = P_old(1,1) - K(1)*P_old(1,1);
        P(1,2) = P_old(1,2) - K(1)*P_old(1,2);
        P(2,1) = P_old(2,1) - K(2)*P_old(1,1);
        P(2,2) = P_old(2,2) - K(2)*P_old(1,2);
    end
    KF.P = P;
end

function ang = wrapTo180(ang)
    ang = mod(ang+180, 360) - 180;
end

function [V,F,C]=makeBodyCube(Lx,Ly,Lz)
    x=[-Lx/2 Lx/2]; y=[-Ly/2 Ly/2]; z=[-Lz/2 Lz/2];
    V=[x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1);
       x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
    F=[1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    C=repmat([0.8 0.8 0.8],6,1); C(1,:)=[0.9 0.3 0.3]; 
end