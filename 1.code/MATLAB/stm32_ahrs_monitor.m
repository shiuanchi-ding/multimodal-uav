clear; clc; close all;

COM_PORT = "COM8";
BAUD     = 115200;

% 建立圖形視窗
fig = figure('Name','STM32 Drone Monitor', 'Color','w', ...
    'KeyPressFcn', @keyPressHandler);

% 建立 3D 軸
ax3d = axes(fig); 
grid(ax3d,'on'); axis(ax3d,'equal'); hold(ax3d,'on');
xlabel(ax3d,'X (Roll)'); ylabel(ax3d,'Y (Pitch)'); zlabel(ax3d,'Z (Yaw)'); 
view(ax3d, 35, 25);
xlim(ax3d,[-0.5 0.5]); ylim(ax3d,[-0.5 0.5]); zlim(ax3d,[-0.5 0.5]);

% 繪製無人機模型
ht = hgtransform('Parent',ax3d);
[cubeV,cubeF,cubeC] = makeBodyCube(0.25, 0.25, 0.05); 
patch('Vertices',cubeV,'Faces',cubeF,'FaceVertexCData',cubeC, ...
      'FaceColor','flat','EdgeColor',[0.2 0.2 0.2], ...
      'Parent',ht,'FaceAlpha',0.9);
% 座標軸箭頭
quiver3(0,0,0,0.4,0,0,'r','LineWidth',2,'Parent',ht); 
quiver3(0,0,0,0,0.4,0,'g','LineWidth',2,'Parent',ht); 
quiver3(0,0,0,0,0,0.4,'b','LineWidth',2,'Parent',ht); 

light(ax3d,'Position',[1 1 -1]); 
lighting(ax3d, 'gouraud'); material(ax3d,'dull');

% 狀態文字
txt_info = text(ax3d, 0, 0, 0.6, 'Connecting...', ...
    'HorizontalAlignment','center', 'FontSize', 12, 'FontWeight','bold');

fprintf('正在連接 %s (%d) ... ', COM_PORT, BAUD);
s = [];
try
    s = serialport(COM_PORT, BAUD);
    configureTerminator(s, "CR/LF");
    flush(s);
    fprintf('成功！\n');
catch ME
    fprintf('失敗！\n錯誤: %s\n', ME.message);
    return;
end

setappdata(fig, 'SerialPort', s);

disp('監控中... 按 "z" 發送歸零指令給 STM32');

while ishandle(fig)
    % 防延遲讀取
    if s.NumBytesAvailable == 0
        drawnow limitrate; 
        continue; 
    end
    
    last_line = "";
    while s.NumBytesAvailable > 0
        temp_line = readline(s);
        if strlength(temp_line) > 5, last_line = temp_line; end
    end
    
    if strlength(last_line) < 5, continue; end
    
    % 解析 CSV: Roll, Pitch, Yaw, AccNorm, Locked
    try
        vals = str2double(split(strtrim(last_line),','));
    catch
        continue;
    end
    
    if numel(vals) < 5 || any(isnan(vals)), continue; end
    
    % 取值
    roll  = vals(1);
    pitch = vals(2);
    yaw   = vals(3);
    acc_norm = vals(4);
    is_locked = vals(5);
    
    % 更新 3D 姿態
    Rx = makehgtform('xrotate', deg2rad(roll));
    Ry = makehgtform('yrotate', deg2rad(pitch));
    Rz = makehgtform('zrotate', deg2rad(yaw));
    set(ht, 'Matrix', Rz * Ry * Rx);
    
    % 狀態顏色
    if is_locked
        status_str = '\color{green}[Locked]';
    else
        status_str = '\color{red}[Calibrating...]';
    end
    
    if abs(acc_norm - 1.0) > 0.1
        acc_str = sprintf('\\color{red}Acc: %.2fg (Vibration)', acc_norm);
    else
        acc_str = sprintf('\\color{black}Acc: %.2fg (OK)', acc_norm);
    end
    
    txt_info.String = sprintf('%s\nRoll: %.1f  Pitch: %.1f  Yaw: %.1f\n%s', ...
        status_str, roll, pitch, yaw, acc_str);
    
    drawnow limitrate;
end

try, clear s; catch, end
disp('連線已斷開');

%% ==== Functions ====
function keyPressHandler(src, evt)
    if strcmpi(evt.Key, 'z')
        s = getappdata(src, 'SerialPort');
        if ~isempty(s)
            writeline(s, 'z'); % 發送 'z' 給 STM32
            fprintf('已發送指令: Zero (Tare)\n');
        end
    end
end

function [V,F,C]=makeBodyCube(Lx,Ly,Lz)
    x=[-Lx/2 Lx/2]; y=[-Ly/2 Ly/2]; z=[-Lz/2 Lz/2];
    V=[x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1);
       x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
    F=[1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    C=repmat([0.8 0.8 0.8],6,1); C(1,:)=[0.9 0.3 0.3]; 
end