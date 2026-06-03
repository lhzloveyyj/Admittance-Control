%% 导纳控制 Simulink 仿真 —— PI + 导纳级联 零力控制
%  架构: F_sensor → [Kp + Ki/s] → [1/(M_d*s+B_d)] → v_des → MotorPlant
%  PI 保证稳态零误差, 导纳层塑造瞬态手感
%  目标: 推拉时传感器读数稳定在0, 用户感觉无水平阻力
clear; clc; close all;
bdclose('all');

%% ==================== 系统参数 ====================

% ---- PI 力控制器 ----
Kp = 3.0;       % 比例增益, 决定瞬态响应快慢
Ki = 6.0;       % 积分增益, 保证稳态零误差 (必须有!)

% ---- 导纳层 (塑造手感) ----
M_d = 0.5;      % 虚拟质量 (kg), 越小越跟手
B_d = 1.0;      % 虚拟阻尼 (N·s/m), 越小阻力感越小

% ---- 被控对象: FOC速度环 + 电机 + 减速器 + 助力轮 ----
%  G(s) = K_sys / (tau_sys*s + 1)
K_sys   = 0.3;      % 速度指令→助力增益 (N / (rad/s)), 需实测标定
tau_sys = 0.02;     % 速度环等效时间常数 (s)

% ---- 传感器信号调理 ----
filt_fc   = 20;     % 低通截止频率 (Hz)
dead_zone = 0.15;   % 死区 (N)
noise_std = 0.05;   % 噪声标准差 (N)
filt_tau  = 1/(2*pi*filt_fc);

% ---- 输出限幅 ----
V_max = 300;        % 最大速度 (rad/s)

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

%% --- 放置模块 ---

% Step 信号源
add_block('simulink/Sources/Step', [model_name '/Step_Fuser']);
set_param([model_name '/Step_Fuser'], ...
    'Time', '0.5', 'Before', '0', 'After', '10');

% Sine 信号源
add_block('simulink/Sources/Sine Wave', [model_name '/Sine_Fuser']);
set_param([model_name '/Sine_Fuser'], ...
    'Amplitude', '5', 'Frequency', num2str(2*pi*0.5), 'Bias', '0');

% 切换开关
add_block('simulink/Signal Routing/Manual Switch', [model_name '/Switch']);

% 力叠加: F_sensor = F_user (+) - F_assist (-)
add_block('simulink/Math Operations/Sum', [model_name '/ForceSum']);
set_param([model_name '/ForceSum'], 'Inputs', '|+-', 'IconShape', 'round');

% 死区
add_block('simulink/Discontinuities/Dead Zone', [model_name '/DeadZone']);
set_param([model_name '/DeadZone'], ...
    'LowerValue', num2str(-dead_zone), 'UpperValue', num2str(dead_zone));

% 低通滤波
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

% PI + 导纳 合并传函:  (Kp*s + Ki) / (M_d*s^2 + B_d*s)
add_block('simulink/Continuous/Transfer Fcn', [model_name '/PI_Admittance']);
set_param([model_name '/PI_Admittance'], ...
    'Numerator', ['[', num2str(Kp), ' ', num2str(Ki), ']'], ...
    'Denominator', ['[', num2str(M_d), ' ', num2str(B_d), ' 0]']);

% 速度限幅
add_block('simulink/Discontinuities/Saturation', [model_name '/VelLimit']);
set_param([model_name '/VelLimit'], ...
    'UpperLimit', num2str(V_max), 'LowerLimit', num2str(-V_max));

% 被控对象
add_block('simulink/Continuous/Transfer Fcn', [model_name '/MotorPlant']);
set_param([model_name '/MotorPlant'], ...
    'Numerator', num2str(K_sys), 'Denominator', ['[', num2str(tau_sys), ' 1]']);

% Scope
add_block('simulink/Sinks/Scope', [model_name '/Scope']);
set_param([model_name '/Scope'], 'NumInputPorts', '5');

% To Workspace
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Fuser']);
set_param([model_name '/WS_Fuser'], 'VariableName', 'Fuser', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Fsensor']);
set_param([model_name '/WS_Fsensor'], 'VariableName', 'Fsensor', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Fassist']);
set_param([model_name '/WS_Fassist'], 'VariableName', 'Fassist', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Vdes']);
set_param([model_name '/WS_Vdes'], 'VariableName', 'Vdes', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name '/WS_Integral']);
set_param([model_name '/WS_Integral'], 'VariableName', 'IntegralOut', 'SaveFormat', 'Array');

% 分离 PI 输出和导纳输出, 便于观察
add_block('simulink/Continuous/Transfer Fcn', [model_name '/PI_Only']);
set_param([model_name '/PI_Only'], ...
    'Numerator', ['[', num2str(Kp), ' ', num2str(Ki), ']'], ...
    'Denominator', '[1 0]');

%% --- 连线 ---

add_line(model_name, 'Step_Fuser/1', 'Switch/1');
add_line(model_name, 'Sine_Fuser/1', 'Switch/2');
add_line(model_name, 'Switch/1', 'ForceSum/1');

add_line(model_name, 'ForceSum/1', 'DeadZone/1');
add_line(model_name, 'DeadZone/1', 'LowPass/1');
add_line(model_name, 'LowPass/1', 'AddNoise/1');
add_line(model_name, 'Noise/1', 'AddNoise/2');
add_line(model_name, 'AddNoise/1', 'PI_Admittance/1');
add_line(model_name, 'PI_Admittance/1', 'VelLimit/1');
add_line(model_name, 'VelLimit/1', 'MotorPlant/1');
add_line(model_name, 'MotorPlant/1', 'ForceSum/2');

% 额外的 PI 输出观测通道
add_line(model_name, 'AddNoise/1', 'PI_Only/1');

% Scope: F_user, F_sensor, F_assist, V_des, PI_out
add_line(model_name, 'Switch/1', 'Scope/1');
add_line(model_name, 'AddNoise/1', 'Scope/2');
add_line(model_name, 'MotorPlant/1', 'Scope/3');
add_line(model_name, 'VelLimit/1', 'Scope/4');
add_line(model_name, 'PI_Only/1', 'Scope/5');

% To Workspace
add_line(model_name, 'Switch/1', 'WS_Fuser/1');
add_line(model_name, 'AddNoise/1', 'WS_Fsensor/1');
add_line(model_name, 'MotorPlant/1', 'WS_Fassist/1');
add_line(model_name, 'VelLimit/1', 'WS_Vdes/1');
add_line(model_name, 'PI_Only/1', 'WS_Integral/1');

Simulink.BlockDiagram.arrangeSystem(model_name);
save_system(model_name);

fprintf('模型 %s 已创建 (PI + 导纳级联).\n', model_name);
fprintf('  PI:  Kp=%.1f, Ki=%.1f\n', Kp, Ki);
fprintf('  导纳: M_d=%.2f, B_d=%.2f\n', M_d, B_d);
fprintf('  被控对象: K_sys=%.2f, tau_sys=%.3f s\n', K_sys, tau_sys);

%% ==================== 系统分析 ====================

fprintf('\n==== 闭环系统分析 (PI + 导纳) ====\n');

% 开环: L(s) = K_sys * (Kp*s + Ki) / [s * (M_d*s + B_d) * (tau_sys*s + 1)]
fprintf('开环传函:\n');
fprintf('  L(s) = %.2f*(%.1f s + %.1f) / [s * (%.2f s + %.2f) * (%.3f s + 1)]\n', ...
    K_sys, Kp, Ki, M_d, B_d, tau_sys);

% 特征方程系数: a3*s^3 + a2*s^2 + a1*s + a0
a3 = M_d * tau_sys;
a2 = M_d + B_d * tau_sys;
a1 = B_d + K_sys * Kp;
a0 = K_sys * Ki;

fprintf('\n特征方程: %.4f s^3 + %.3f s^2 + %.3f s + %.2f = 0\n', a3, a2, a1, a0);

% 求闭环极点
poles = roots([a3 a2 a1 a0]);
fprintf('闭环极点:\n');
for i = 1:3
    if imag(poles(i)) == 0
        fprintf('  s%d = %.2f  (实极点, τ=%.1f ms)\n', i, poles(i), -1000/poles(i));
    else
        fprintf('  s%d = %.2f ± %.2f j  (ω_n=%.1f, ζ=%.2f)\n', ...
            i, real(poles(i)), abs(imag(poles(i))), ...
            abs(poles(i)), -real(poles(i))/abs(poles(i)));
    end
end

% 稳态误差: 阶跃→0, 斜坡→B_d/(K_sys*Ki)
fprintf('\n稳态分析:\n');
fprintf('  阶跃推力: F_sensor_ss = 0  (PI的I项保证)\n');
fprintf('  斜坡推力: F_sensor_ss = B_d / (K_sys * Ki) = %.4f N/(N/s)\n', B_d/(K_sys*Ki));

%% ==================== 运行仿真 ====================

% --- 测试1: 阶跃推力 ---
set_param([model_name '/Switch'], 'sw', '1');
fprintf('\n=== 测试1: 阶跃推力 10N @ 0.5s ===\n');
simOut1 = sim(model_name, 'StopTime', '5');

t1  = simOut1.tout;
Fu1 = simOut1.Fuser;
Fs1 = simOut1.Fsensor;
Fa1 = simOut1.Fassist;
Vd1 = simOut1.Vdes;

fprintf('  t=5s: F_user=%.2f  F_assist=%.2f  F_sensor=%.4f N  V_des=%.2f\n', ...
    Fu1(end), Fa1(end), Fs1(end), Vd1(end));
fprintf('  峰值|F_sensor|=%.3f N  (瞬态)\n', max(abs(Fs1)));

% --- 测试2: 正弦推拉 ---
set_param([model_name '/Switch'], 'sw', '0');
fprintf('\n=== 测试2: 正弦推力 ±5N @ 0.5Hz ===\n');
simOut2 = sim(model_name, 'StopTime', '10');

t2  = simOut2.tout;
Fu2 = simOut2.Fuser;
Fs2 = simOut2.Fsensor;
Fa2 = simOut2.Fassist;
Vd2 = simOut2.Vdes;

fprintf('  RMS F_sensor=%.4f N  峰值|F_sensor|=%.3f N\n', rms(Fs2), max(abs(Fs2)));

%% ==================== 绘图 ====================

% 图1: 阶跃响应
figure('Name', 'PI+Admittance - Step 10N', 'Position', [50 80 1000 750]);

subplot(5,1,1);
plot(t1, Fu1, 'b', t1, Fa1, 'r', 'LineWidth', 1.5);
ylabel('Force (N)'); legend('F_{user}', 'F_{assist}', 'Location', 'best');
title(sprintf('PI+导纳 阶跃10N (Kp=%.1f, Ki=%.1f, M_d=%.2f, B_d=%.2f)', Kp, Ki, M_d, B_d));
grid on;

subplot(5,1,2);
plot(t1, Fs1, 'k', 'LineWidth', 1.5);
yline(0, '--'); yline(dead_zone, 'g:'); yline(-dead_zone, 'g:');
ylabel('F_{sensor} (N)'); legend('Sensor', 'Target=0', 'Location', 'best');
grid on;

subplot(5,1,3);
plot(t1, Fu1 - Fa1, 'm', 'LineWidth', 1);
ylabel('Error (N)');
grid on;

subplot(5,1,4);
plot(t1, Vd1, 'Color', [0 0.5 0], 'LineWidth', 1.5);
yline(V_max, 'r--'); yline(-V_max, 'r--');
ylabel('V_{des} (rad/s)');
grid on;

subplot(5,1,5);
plot(t1, simOut1.IntegralOut, 'Color', [0.6 0.2 0.6], 'LineWidth', 1.2);
yline(0, '--');
ylabel('PI Integ Out'); xlabel('Time (s)');
grid on;

% 图2: 正弦响应
figure('Name', 'PI+Admittance - Sine +/-5N 0.5Hz', 'Position', [100 130 1000 750]);

subplot(5,1,1);
plot(t2, Fu2, 'b', t2, Fa2, 'r', 'LineWidth', 1.5);
ylabel('Force (N)'); legend('F_{user}', 'F_{assist}', 'Location', 'best');
title(sprintf('PI+导纳 正弦±5N 0.5Hz (Kp=%.1f, Ki=%.1f)', Kp, Ki));
grid on;

subplot(5,1,2);
plot(t2, Fs2, 'k', 'LineWidth', 1.5);
yline(0, '--');
ylabel('F_{sensor} (N)');
grid on;

subplot(5,1,3);
plot(t2, Fu2 - Fa2, 'm', 'LineWidth', 1);
ylabel('Error (N)');
grid on;

subplot(5,1,4);
plot(t2, Vd2, 'Color', [0 0.5 0], 'LineWidth', 1.5);
ylabel('V_{des} (rad/s)');
grid on;

subplot(5,1,5);
plot(t2, simOut2.IntegralOut, 'Color', [0.6 0.2 0.6], 'LineWidth', 1.2);
ylabel('PI Integ Out'); xlabel('Time (s)');
grid on;

%% ==================== B_d 参数扫描 ====================

fprintf('\n==== B_d 参数扫描 (阶跃 10N) ====\n');
Bd_values = [0.3, 0.5, 1.0, 2.0];
figure('Name', 'B_d Sweep', 'Position', [150 180 1000 600]);

colors = lines(length(Bd_values));
set_param([model_name '/Switch'], 'sw', '1');

for i = 1:length(Bd_values)
    set_param([model_name '/PI_Admittance'], ...
        'Denominator', ['[', num2str(M_d), ' ', num2str(Bd_values(i)), ' 0]']);

    simOut = sim(model_name, 'StopTime', '3');

    subplot(2,2,1);
    plot(simOut.tout, simOut.Fsensor, 'Color', colors(i,:), 'LineWidth', 1.2); hold on;
    ylabel('F_{sensor} (N)'); title('力传感器响应'); grid on;

    subplot(2,2,2);
    plot(simOut.tout, simOut.Fassist, 'Color', colors(i,:), 'LineWidth', 1.2); hold on;
    ylabel('F_{assist} (N)'); title('电机助力'); grid on;

    subplot(2,2,3);
    plot(simOut.tout, simOut.Vdes, 'Color', colors(i,:), 'LineWidth', 1.2); hold on;
    ylabel('V_{des} (rad/s)'); xlabel('Time (s)'); title('速度指令'); grid on;

    subplot(2,2,4);
    plot(simOut.tout, simOut.IntegralOut, 'Color', colors(i,:), 'LineWidth', 1.2); hold on;
    ylabel('Integ Out'); xlabel('Time (s)'); title('PI积分项'); grid on;

    fprintf('  B_d=%.1f: F_sensor_ss=%.4f N, 峰值=%.3f N, settling~%.2f s\n', ...
        Bd_values(i), simOut.Fsensor(end), max(abs(simOut.Fsensor)), ...
        simOut.tout(find(abs(simOut.Fsensor) < 0.2, 1, 'first')));
end

for sp = 1:4
    subplot(2,2,sp);
    leg = legend(arrayfun(@(b) sprintf('B_d=%.1f', b), Bd_values, ...
        'UniformOutput', false), 'Location', 'best');
end

% 恢复
set_param([model_name '/PI_Admittance'], ...
    'Denominator', ['[', num2str(M_d), ' ', num2str(B_d), ' 0]']);

%% ==================== 操作提示 ====================

fprintf('\n操作指南:\n');
fprintf('  open_system(''%s'')         查看模型\n', model_name);
fprintf('  PI_Admittance 块:  Numerator = [Kp, Ki],  Denominator = [M_d, B_d, 0]\n');
fprintf('  MotorPlant 块:     实测后替换 K_sys 和 tau_sys\n');
fprintf('  DeadZone 块:       调节死区阈值\n');
fprintf('  修改参数后重新运行脚本即可\n');

fprintf('\n推荐整定顺序:\n');
fprintf('  1. 先调 Kp: 推拉时力传感器有快速响应, 不振荡\n');
fprintf('  2. 再调 Ki: 推住不动时传感器读数缓慢归零\n');
fprintf('  3. 调 M_d: 启停是否跟手 (越小越跟手)\n');
fprintf('  4. 调 B_d: 阻力感大小 (越小越省力)\n');
