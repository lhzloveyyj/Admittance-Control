%% 导纳控制 Simulink 仿真 —— 完整 FOC 层级模型
%  架构 (对应实际 FOC 固件):
%    F_sensor → [Force_PI] → [Admittance] → v_des
%      → [Vel_PI] → Iq_ref → [Current_Loop] → Iq → [Motor_Mech] → ω
%      → [Force_Conv] → F_assist → 闭环反馈
%
%  参数说明:
%    力控外环:   Kp_f, Ki_f, M_d, B_d   (导纳控制, 200Hz)
%    FOC 速度环:  Kp_v, Ki_v             (PI 速度控制, 1kHz)
%    FOC 电流环:  tau_i                  (一阶等效, 16kHz)
%    电机本体:    J, B, K_t             (电气+机械参数)
%    传动机构:    N, r_wheel, eta       (减速比, 轮径, 效率)
clear; clc; close all;
bdclose('all');

%% ==================== 系统参数 ====================

% ===== 力控外环: PI 力控制 + 导纳 =====
Kp_f = 3.0;         % 力环比例增益
Ki_f = 6.0;         % 力环积分增益 (保证传感器归零)
M_d  = 0.5;         % 虚拟质量 (kg)
B_d  = 1.0;         % 虚拟阻尼 (N·s/m)

% ===== FOC 速度环 PI =====
Kp_v = 0.05;        % 速度环比例增益 (A / (rad/s))
Ki_v = 2.0;         % 速度环积分增益 (A / rad)

% ===== FOC 电流环 (等效一阶) =====
tau_i = 0.001;      % 电流环时间常数 (s), ~1kHz 带宽

% ===== 电机本体 =====
J   = 5e-6;         % 转子转动惯量 (kg·m²)
B_m = 2e-4;         % 粘性阻尼系数 (N·m·s/rad)
K_t = 0.04;         % 转矩常数 (N·m/A)

% ===== 传动机构 =====
N_gear  = 50;       % 减速比
r_wheel = 0.025;    % 轮子半径 (m)
eta     = 0.85;     % 传动效率

% ===== 力换算 =====
%  F_assist = 电机转矩 * N * η / r
%           = K_t * Iq * N * η / r
K_force = K_t * N_gear * eta / r_wheel;  % N/A, 电流→助力增益

% ===== 传感器信号调理 =====
filt_fc   = 20;     % 低通截止频率 (Hz)
dead_zone = 0.15;   % 死区 (N)
noise_std = 0.05;   % 噪声标准差 (N)
filt_tau  = 1/(2*pi*filt_fc);

% ===== 限幅 =====
V_max   = 300;      % 最大速度指令 (rad/s)
Iq_max  = 5;        % 最大 Iq 电流 (A)

fprintf('========== 系统参数 ==========\n');
fprintf('力控外环:  Kp_f=%.1f, Ki_f=%.1f, M_d=%.2f, B_d=%.2f\n', Kp_f, Ki_f, M_d, B_d);
fprintf('FOC 速度环: Kp_v=%.3f, Ki_v=%.1f\n', Kp_v, Ki_v);
fprintf('FOC 电流环: tau_i=%.1f ms\n', tau_i*1000);
fprintf('电机本体:   J=%.0e, B_m=%.0e, K_t=%.3f\n', J, B_m, K_t);
fprintf('传动机构:   N=%d, r=%.3f m, η=%.0f%%\n', N_gear, r_wheel, eta*100);
fprintf('力换算:     K_force=%.1f N/A (1A → %.1f N 助力)\n', K_force, K_force);

%% ==================== 创建模型 ====================

model_name = 'admittance_control';

if exist([model_name '.slx'], 'file')
    delete([model_name '.slx']);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.001');
set_param(model_name, 'StopTime', '10');
set_param(model_name, 'SignalLogging', 'on');
set_param(model_name, 'SignalLoggingName', 'logsout');

%% --- 放置所有模块 ---

% 测试信号: 阶跃推力
add_block('simulink/Sources/Step', [model_name '/Step_Fuser']);
set_param([model_name '/Step_Fuser'], 'Time', '0.5', 'Before', '0', 'After', '10');

% 测试信号: 正弦推力
add_block('simulink/Sources/Sine Wave', [model_name '/Sine_Fuser']);
set_param([model_name '/Sine_Fuser'], ...
    'Amplitude', '5', 'Frequency', num2str(2*pi*0.5), 'Bias', '0');

% 切换开关
add_block('simulink/Signal Routing/Manual Switch', [model_name '/Switch']);

% ---- 传感器部分 ----
% F_sensor = F_human - F_motor
add_block('simulink/Math Operations/Sum', [model_name '/ForceSum']);
set_param([model_name '/ForceSum'], 'Inputs', '|+-', 'IconShape', 'round');

% 死区
add_block('simulink/Discontinuities/Dead Zone', [model_name '/DeadZone']);
set_param([model_name '/DeadZone'], ...
    'LowerValue', num2str(-dead_zone), 'UpperValue', num2str(dead_zone));

% 20Hz 低通滤波
add_block('simulink/Continuous/Transfer Fcn', [model_name '/LowPass']);
set_param([model_name '/LowPass'], ...
    'Numerator', '[1]', 'Denominator', ['[', num2str(filt_tau), ' 1]']);

% 噪声
add_block('simulink/Sources/Random Number', [model_name '/Noise']);
set_param([model_name '/Noise'], ...
    'Mean', '0', 'Variance', num2str(noise_std^2), ...
    'SampleTime', '0.001', 'Seed', '42');

add_block('simulink/Math Operations/Sum', [model_name '/AddNoise']);
set_param([model_name '/AddNoise'], 'Inputs', '|++', 'IconShape', 'round');

% ---- PI 力控制器 ----
add_block('simulink/Continuous/PID Controller', [model_name '/Force_PI']);
set_param([model_name '/Force_PI'], ...
    'Controller', 'PI', ...
    'P', num2str(Kp_f), 'I', num2str(Ki_f), ...
    'D', '0', 'N', '100');

% ---- 导纳滤波器 1/(M_d*s + B_d) ----
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Admittance']);
set_param([model_name '/Admittance'], ...
    'Numerator', '[1]', 'Denominator', ['[', num2str(M_d), ' ', num2str(B_d), ']']);

% 速度指令限幅
add_block('simulink/Discontinuities/Saturation', [model_name '/VelLimit']);
set_param([model_name '/VelLimit'], ...
    'UpperLimit', num2str(V_max), 'LowerLimit', num2str(-V_max));

% ---- FOC 速度环 PI (外环输出 v_des, 反馈 ω_actual) ----
add_block('simulink/Math Operations/Sum', [model_name '/VelErr']);
set_param([model_name '/VelErr'], 'Inputs', '|+-', 'IconShape', 'round');

add_block('simulink/Continuous/PID Controller', [model_name '/Vel_PI']);
set_param([model_name '/Vel_PI'], ...
    'Controller', 'PI', ...
    'P', num2str(Kp_v), 'I', num2str(Ki_v), ...
    'D', '0', 'N', '100', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', num2str(Iq_max), ...
    'LowerSaturationLimit', num2str(-Iq_max));

% ---- FOC 电流环 (等效一阶) ----
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Current_Loop']);
set_param([model_name '/Current_Loop'], ...
    'Numerator', '[1]', 'Denominator', ['[', num2str(tau_i), ' 1]']);

% ---- 电机: 转矩生成 ----
add_block('simulink/Math Operations/Gain', [model_name '/Torque_Gain']);
set_param([model_name '/Torque_Gain'], 'Gain', num2str(K_t));

% ---- 电机: 机械动力学 ω = Torque / (J*s + B) ----
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Motor_Mech']);
set_param([model_name '/Motor_Mech'], ...
    'Numerator', '[1]', 'Denominator', ['[', num2str(J), ' ', num2str(B_m), ']']);

% ---- 传动机构: 转矩→轮子助力 ----
add_block('simulink/Math Operations/Gain', [model_name '/Force_Conv']);
set_param([model_name '/Force_Conv'], ...
    'Gain', num2str(N_gear * eta / r_wheel));

% ---- 观测与导出 ----
add_block('simulink/Sinks/Scope', [model_name '/Scope']);
set_param([model_name '/Scope'], 'NumInputPorts', '6');

add_block('simulink/Sinks/To Workspace', [model_name '/WS_Fuser']);
set_param([model_name '/WS_Fuser'], 'VariableName', 'Fuser', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Fsensor']);
set_param([model_name '/WS_Fsensor'], 'VariableName', 'Fsensor', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Fassist']);
set_param([model_name '/WS_Fassist'], 'VariableName', 'Fassist', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Vdes']);
set_param([model_name '/WS_Vdes'], 'VariableName', 'Vdes', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Omega']);
set_param([model_name '/WS_Omega'], 'VariableName', 'Omega', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Iq']);
set_param([model_name '/WS_Iq'], 'VariableName', 'Iq', 'SaveFormat', 'Array');

%% --- 连线 ---

% 测试信号
add_line(model_name, 'Step_Fuser/1', 'Switch/1');
add_line(model_name, 'Sine_Fuser/1', 'Switch/2');

% 传感器链: Switch → ForceSum → DeadZone → LowPass → AddNoise
add_line(model_name, 'Switch/1', 'ForceSum/1');
add_line(model_name, 'ForceSum/1', 'DeadZone/1');
add_line(model_name, 'DeadZone/1', 'LowPass/1');
add_line(model_name, 'LowPass/1', 'AddNoise/1');
add_line(model_name, 'Noise/1', 'AddNoise/2');

% 力控制器链: AddNoise → Force_PI → Admittance → VelLimit
add_line(model_name, 'AddNoise/1', 'Force_PI/1');
add_line(model_name, 'Force_PI/1', 'Admittance/1');
add_line(model_name, 'Admittance/1', 'VelLimit/1');

% 速度环: VelLimit → VelErr(+) → Vel_PI → Current_Loop → Torque_Gain
add_line(model_name, 'VelLimit/1', 'VelErr/1');
add_line(model_name, 'VelErr/1', 'Vel_PI/1');
add_line(model_name, 'Vel_PI/1', 'Current_Loop/1');
add_line(model_name, 'Current_Loop/1', 'Torque_Gain/1');

% 电机机械: Torque → Motor_Mech → ω
add_line(model_name, 'Torque_Gain/1', 'Motor_Mech/1');

% 速度反馈: Motor_Mech → VelErr(-)
add_line(model_name, 'Motor_Mech/1', 'VelErr/2');

% 力换算: Torque → Force_Conv → F_assist → ForceSum(-) 闭环
add_line(model_name, 'Torque_Gain/1', 'Force_Conv/1');
add_line(model_name, 'Force_Conv/1', 'ForceSum/2');

% ===== Scope 6通道 =====
% Ch1: F_user, Ch2: F_sensor, Ch3: F_assist
% Ch4: v_des(蓝色虚线) vs ω_actual(红色虚线), Ch5: Iq_ref, Ch6: Force_PI_out
add_line(model_name, 'Switch/1', 'Scope/1');        % F_user
add_line(model_name, 'AddNoise/1', 'Scope/2');       % F_sensor
add_line(model_name, 'Force_Conv/1', 'Scope/3');     % F_assist
add_line(model_name, 'VelLimit/1', 'Scope/4');       % v_des
add_line(model_name, 'Motor_Mech/1', 'Scope/5');     % ω_actual
add_line(model_name, 'Current_Loop/1', 'Scope/6');   % Iq

% ===== To Workspace =====
add_line(model_name, 'Switch/1', 'WS_Fuser/1');
add_line(model_name, 'AddNoise/1', 'WS_Fsensor/1');
add_line(model_name, 'Force_Conv/1', 'WS_Fassist/1');
add_line(model_name, 'VelLimit/1', 'WS_Vdes/1');
add_line(model_name, 'Motor_Mech/1', 'WS_Omega/1');
add_line(model_name, 'Current_Loop/1', 'WS_Iq/1');

% 自动排列
Simulink.BlockDiagram.arrangeSystem(model_name);
save_system(model_name);

fprintf('\n模型 %s 已创建.\n', model_name);
fprintf('信号链: F_user → ForceSum → DeadZone→LowPass→AddNoise\n');
fprintf('         → Force_PI → Admittance → VelLimit\n');
fprintf('         → VelErr → Vel_PI → Current_Loop → Torque→Motor_Mech→ω\n');
fprintf('                                    └→ Force_Conv → F_assist ─(反馈)─┘\n');
fprintf('                              Motor_Mech→ω ─(速度反馈)─→ VelErr(-)\n');

%% ==================== 运行仿真 ====================

% --- 测试1: 阶跃推力 ---
set_param([model_name '/Switch'], 'sw', '1');
fprintf('\n=== 测试1: 阶跃推力 10N @ 0.5s ===\n');
simOut1 = sim(model_name, 'StopTime', '5');

t1  = simOut1.tout;
Fs1 = simOut1.Fsensor;
Fa1 = simOut1.Fassist;
Vd1 = simOut1.Vdes;
Om1 = simOut1.Omega;
Iq1 = simOut1.Iq;

fprintf('  t=5s: F_sensor=%.4f N, F_assist=%.2f N, v_des=%.1f, ω=%.1f rad/s, Iq=%.3f A\n', ...
    Fs1(end), Fa1(end), Vd1(end), Om1(end), Iq1(end));

% --- 测试2: 正弦推拉 ---
set_param([model_name '/Switch'], 'sw', '0');
fprintf('\n=== 测试2: 正弦推力 ±5N @ 0.5Hz ===\n');
simOut2 = sim(model_name, 'StopTime', '10');

t2  = simOut2.tout;
Fs2 = simOut2.Fsensor;
Fa2 = simOut2.Fassist;
Vd2 = simOut2.Vdes;
Om2 = simOut2.Omega;
Iq2 = simOut2.Iq;

fprintf('  RMS F_sensor=%.4f N, 峰值|F_sensor|=%.3f N\n', rms(Fs2), max(abs(Fs2)));

%% ==================== 绘图 ====================

% 图1: 阶跃响应
figure('Name', 'Step 10N - Full FOC Model', 'Position', [50 50 1100 800]);

subplot(6,1,1);
plot(t1, simOut1.Fuser, 'b', t1, Fa1, 'r', 'LineWidth', 1.5);
ylabel('Force (N)');
legend('F_{user}', 'F_{assist}');
title(sprintf('阶跃10N (Kp_f=%.1f Ki_f=%.1f M_d=%.2f B_d=%.2f | 速度环 Kp_v=%.3f Ki_v=%.1f)', ...
    Kp_f, Ki_f, M_d, B_d, Kp_v, Ki_v));
grid on;

subplot(6,1,2);
plot(t1, Fs1, 'k', 'LineWidth', 1.5);
yline(0, '--'); yline(dead_zone, 'g:'); yline(-dead_zone, 'g:');
ylabel('F_{sensor} (N)'); legend('Sensor');
grid on;

subplot(6,1,3);
plot(t1, Fa1, 'r', 'LineWidth', 1);
ylabel('F_{assist} (N)');
grid on;

subplot(6,1,4);
plot(t1, Vd1, 'b', t1, Om1, 'r--', 'LineWidth', 1);
ylabel('Speed (rad/s)');
legend('v_{des} (指令)', '\omega_{actual} (实际)', 'Location', 'best');
grid on;

subplot(6,1,5);
plot(t1, Iq1, 'Color', [0 0.5 0], 'LineWidth', 1);
yline(Iq_max, 'r--'); yline(-Iq_max, 'r--');
ylabel('I_q (A)'); legend('电流环输出');
grid on;

subplot(6,1,6);
plot(t1, simOut1.Fuser - Fa1, 'm', 'LineWidth', 1);
ylabel('F error (N)'); xlabel('Time (s)');
grid on;

% 图2: 正弦响应
figure('Name', 'Sine +/-5N 0.5Hz - Full FOC Model', 'Position', [100 100 1100 800]);

subplot(6,1,1);
plot(t2, simOut2.Fuser, 'b', t2, Fa2, 'r', 'LineWidth', 1.5);
ylabel('Force (N)'); legend('F_{user}', 'F_{assist}');
grid on;

subplot(6,1,2);
plot(t2, Fs2, 'k', 'LineWidth', 1.5);
yline(0, '--');
ylabel('F_{sensor} (N)');
grid on;

subplot(6,1,3);
plot(t2, Fa2, 'r', 'LineWidth', 1);
ylabel('F_{assist} (N)');
grid on;

subplot(6,1,4);
plot(t2, Vd2, 'b', t2, Om2, 'r--', 'LineWidth', 1);
ylabel('Speed (rad/s)'); legend('v_{des}', '\omega_{actual}', 'Location', 'best');
grid on;

subplot(6,1,5);
plot(t2, Iq2, 'Color', [0 0.5 0], 'LineWidth', 1);
ylabel('I_q (A)');
grid on;

subplot(6,1,6);
plot(t2, simOut2.Fuser - Fa2, 'm', 'LineWidth', 1);
ylabel('F error (N)'); xlabel('Time (s)');
grid on;

%% ==================== 操作提示 ====================

fprintf('\n操作指南:\n');
fprintf('  open_system(''%s'')        查看完整模型\n', model_name);
fprintf('  模型中的模块对应你 FOC 固件的结构:\n');
fprintf('    Force_PI      → 力外环 PI  (200Hz)\n');
fprintf('    Admittance    → 导纳滤波器 (塑造手感)\n');
fprintf('    Vel_PI        → FOC 速度环 PI (1kHz)\n');
fprintf('    Current_Loop  → FOC 电流环 (16kHz, 等效简化)\n');
fprintf('    Torque_Gain   → K_t 转矩常数\n');
fprintf('    Motor_Mech    → 电机机械方程 J*dω/dt + B*ω = torque\n');
fprintf('    Force_Conv    → 减速器+轮子 转矩→力\n');
fprintf('    VelErr        → 速度环误差 (v_des - ω_actual)\n');
fprintf('\n参数整定顺序:\n');
fprintf('  1. 先调 Force_PI  (Kp_f, Ki_f) → 力传感器归零\n');
fprintf('  2. 再调 Admittance (M_d, B_d)   → 手感\n');
fprintf('  3. 模拟你 FOC 的 Vel_PI、Current_Loop 参数可直接填入\n');
fprintf('  4. Motor_Mech 替换为你的电机 J, B_m 实测值\n');
