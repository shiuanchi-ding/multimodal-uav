clc;clear;

% 位置命令
sim_time = 100; % 總模擬時間

% 定義時間軸: [開始, 10秒前, 10秒後, 結束]
t_axis = [0, 10, 10.001, sim_time]; 

% 1. 定義 Stick Input (Pitch, Roll)
% 邏輯: 0~10秒維持 [1, 1], 10秒後瞬間變 [0, 0]
stick_data = [0.3, 0.3;   % t = 0
              0.3, 0.3;   % t = 10
              0, 0;   % t = 10.001
              0, 0];  % t = end

% 轉換為 Simulink 看得懂的 timeseries 物件
Stick_Input_Signal = timeseries(stick_data, t_axis);

% 2. 定義 Vz cmd (Vertical Speed)
% 邏輯: 0~10秒維持 1, 10秒後變成 0
vz_data = [0.5; 0.5; 0; 0];
Vz_cmd_Signal = timeseries(vz_data, t_axis);
X=10;
Y=10;
Z=10;

% 高度控制器
Kvzp=10;Kvzi=0.01;Kvzd=0.1;
% 水平位置控制器
Kposp=1.5; Kposi=0.1;
Kvxp=5; Kvxi=0.001; Kvxd=10;
Kvyp=5; Kvyi=0.001; Kvyd=10;
% 姿態控制器
Kvp=1; Kvi=0; Kvd=0;
% Kvrollp=8; Kvrolli=0.1; Kvrolld=0.1; Kvrolln=100;
% Kvpitchp=22; Kvpitchi=0.015; Kvpitchd=0.015; Kvpitchn=100;
% Kvyawp=30; Kvyawi=0.1; Kvyawd=0.8; Kvyawn=100;

g=9.81;

% 無人機參數
m=2; 
Ix=0.0095;
Iy=0.0095;
Iz=0.019;
cT=1.4865e-7;
cM=2.925e-9;
Tm=0.03;
CR=80.584;
d_x = 0.08257;
d_y = 0.13505;
K=cM/cT;
offset_x = 0.01;  % 重心向前移 2 公分
offset_y = 0.01;  % 重心向左移 1 公分

wb=976.2;

% 推力轉換矩陣參數


x1 =  d_x;
x2 =  d_x;
x3 = -d_x;
x4 = -d_x;


y1 =  d_y;  % M1 左前 (Positive y)
y2 = -d_y;  % M2 右前 (Negative y)
y3 = -d_y;  % M3 右後 (Negative y)
y4 =  d_y;  % M4 左後 (Positive y)

% 建立物理矩陣 B 
B = [
    1,   1,   1,   1;      % Thrust
    y1,  y2,  y3,  y4;     % Roll (Right Bank = +)
    -x1,  -x2,  -x3,  -x4;     % Pitch (Nose Up = +)
    K,   -K,  K,   -K      % Yaw (CCW = +) *假設M1是順時針轉
];

% 計算控制器矩陣
M = inv(B);

% 計算干擾力矩
tau_phi_offset = -offset_y * m * g; 
tau_theta_offset = offset_x * m * g;
tau_psi_offset = 0;

