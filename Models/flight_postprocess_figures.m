%% =========================================================
% flight_postprocess_figures.m
% Main.slx 후처리
% =========================================================

close all; clc;

%% 1) 시뮬레이션 실행
simOut = sim('Main.slx');

%% 2) 워크스페이스 신호 불러오기 + 숫자 배열 변환
t          = localToColumn(simOut.t);
XYZ_E      = localToMatrix(simOut.XYZ_E);
z_E        = localToColumn(simOut.z_E);      % 새로 추가
v_B        = localToMatrix(simOut.v_B);
V_rel      = localToMatrix(simOut.V_rel);
F_thrust_B = localToMatrix(simOut.F_thrust_B);
mass_cur   = localToColumn(simOut.mass_current);
rho        = localToColumn(simOut.rho);
Q_ts       = localToColumn(simOut.Q);
a_B        = localToMatrix(simOut.a_B);      % 새로 추가
F_aero_B   = localToMatrix(simOut.F_aero_B);
M_aero_B   = localToMatrix(simOut.M_aero_B);

%% 3) 길이 / 형상 정리
disp('===== Signal sizes after conversion =====');
disp(['size(t)          = ', mat2str(size(t))]);
disp(['size(XYZ_E)      = ', mat2str(size(XYZ_E))]);
disp(['size(z_E)        = ', mat2str(size(z_E))]);
disp(['size(v_B)        = ', mat2str(size(v_B))]);
disp(['size(V_rel)      = ', mat2str(size(V_rel))]);
disp(['size(F_thrust_B) = ', mat2str(size(F_thrust_B))]);
disp(['size(mass_cur)   = ', mat2str(size(mass_cur))]);
disp(['size(rho)        = ', mat2str(size(rho))]);
disp(['size(Q_ts)       = ', mat2str(size(Q_ts))]);
disp(['size(a_B)        = ', mat2str(size(a_B))]);
disp(['size(F_aero_B)   = ', mat2str(size(F_aero_B))]);
disp(['size(M_aero_B)   = ', mat2str(size(M_aero_B))]);

N = min([ ...
    length(t), ...
    size(XYZ_E,1), ...
    length(z_E), ...
    size(v_B,1), ...
    size(V_rel,1), ...
    size(F_thrust_B,1), ...
    size(F_aero_B,1), ...
    size(M_aero_B,1), ...
    length(mass_cur), ...
    length(rho), ...
    length(Q_ts), ...
    size(a_B,1) ]);

t          = t(1:N);
XYZ_E      = XYZ_E(1:N,:);
z_E        = z_E(1:N);
v_B        = v_B(1:N,:);
V_rel      = V_rel(1:N,:);
F_thrust_B = F_thrust_B(1:N,:);
mass_cur   = mass_cur(1:N);
rho        = rho(1:N);
Q_ts       = Q_ts(1:N);
a_B        = a_B(1:N,:);
F_aero_B   = F_aero_B(1:N,:);
M_aero_B   = M_aero_B(1:N,:);

if size(XYZ_E,2) < 3
    error('XYZ_E가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(XYZ_E)));
end
if size(v_B,2) < 3
    error('v_B가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(v_B)));
end
if size(V_rel,2) < 3
    error('V_rel가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(V_rel)));
end
if size(F_thrust_B,2) < 3
    error('F_thrust_B가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(F_thrust_B)));
end
if size(a_B,2) < 3
    error('a_B가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(a_B)));
end
if size(F_aero_B,2) < 3
    error('F_aero_B가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(F_aero_B)));
end
if size(M_aero_B,2) < 3
    error('M_aero_B가 Nx3 형식이 아닙니다. 현재 크기: %s', mat2str(size(M_aero_B)));
end

%% 4) 기본 전처리
g0 = 9.80665;

x = XYZ_E(:,1);
y = XYZ_E(:,2);

% z_E 직접 사용
alt = z_E;

% 혹시 부호가 반대로 저장되었으면 자동 보정
if max(alt) < abs(min(alt))
    alt = -alt;
end

alt_plot = alt;
alt_plot(alt_plot < 0) = 0;

downrange = sqrt(x.^2 + y.^2);

%% 5) 연소구간 / 관성구간 분리
thrust_mag = sqrt(sum(F_thrust_B(:,1:3).^2, 2));
thrust_threshold = max(thrust_mag) * 0.05;

isBurn = thrust_mag > thrust_threshold;
burn_end_idx = find(isBurn, 1, 'last');

if isempty(burn_end_idx)
    burn_end_idx = round(length(t)/4);
end

burn_end_time = t(burn_end_idx);

% 경계점 겹치게 포함
idx_burn  = 1:burn_end_idx;
idx_coast = burn_end_idx:length(t);

%% 6) 동압
q_raw = Q_ts;

if max(abs(q_raw)) > 100
    q_kPa = q_raw / 1000;
else
    q_kPa = q_raw;
end

[qmax, idx_qmax] = max(q_kPa);

%% 7) 기체 축 가속도 (Simulink 내부 계산값 직접 사용)
ax_g = a_B(:,1) / g0;
ay_g = a_B(:,2) / g0;
az_g = a_B(:,3) / g0;

[axmax_g, idx_axmax] = max(ax_g);

%% 8) 최대고도 / 착지점 계산
[alt_max, idx_altmax] = max(alt_plot);

idx_land = find(alt <= 0 & t > burn_end_time, 1, 'first');
if isempty(idx_land)
    idx_land = length(t);
end

landing_time = t(idx_land);
landing_x = x(idx_land);
landing_y = y(idx_land);
landing_downrange = downrange(idx_land);

%{
%% Debug Figure. Aerodynamic force / moment-> 오류 해결 시 지우기
figure('Color','w');
plot(t, F_aero_B(:,1), 'LineWidth', 1.2); grid on;
xlabel('Time [sec]'); ylabel('Force [N]');
title('F_{aero,B,x}');

figure('Color','w');
plot(t, F_aero_B(:,2), 'LineWidth', 1.2); grid on;
xlabel('Time [sec]'); ylabel('Force [N]');
title('F_{aero,B,y}');

figure('Color','w');
plot(t, F_aero_B(:,3), 'LineWidth', 1.2); grid on;
xlabel('Time [sec]'); ylabel('Force [N]');
title('F_{aero,B,z}');

figure('Color','w');
plot(t, M_aero_B(:,1), 'LineWidth', 1.2); grid on;
xlabel('Time [sec]'); ylabel('Moment [N·m]');
title('M_{aero,B,x}');

figure('Color','w');
plot(t, M_aero_B(:,2), 'LineWidth', 1.2); grid on;
xlabel('Time [sec]'); ylabel('Moment [N·m]');
title('M_{aero,B,y}');

figure('Color','w');
plot(t, M_aero_B(:,3), 'LineWidth', 1.2); grid on;
xlabel('Time [sec]'); ylabel('Moment [N·m]');
title('M_{aero,B,z}');
%}

%% 9) CSV 저장
landing_table = table( ...
    landing_time, landing_x, landing_y, landing_downrange, ...
    'VariableNames', {'time_s','x_m','y_m','downrange_m'});

writetable(landing_table, 'landing_splash_point.csv');

%% =========================================================
% Figure 1. Dynamic Pressure vs Time
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot(t(idx_burn),  q_kPa(idx_burn),  'r-', 'LineWidth', 1.5);
plot(t(idx_coast), q_kPa(idx_coast), 'k-', 'LineWidth', 1.5);

plot(t(idx_qmax), qmax, 'ko', 'MarkerFaceColor', [0.2 0.2 0.1], 'MarkerSize', 6);
text(t(idx_qmax), qmax, sprintf('  X %.3f\n  Y %.5f', t(idx_qmax), qmax), ...
    'FontSize', 11, ...
    'Color', [0 0.45 0.8], ...
    'BackgroundColor', [0.93 0.93 0.96], ...
    'Margin', 8);

xlabel('Time [sec]', 'FontSize', 14);
ylabel('Dynamic pressure [kPa]', 'FontSize', 14);
legend('burning', 'coasting', 'Location', 'northeast');
set(gca, 'FontSize', 13);

%% =========================================================
% Figure 2. Altitude vs Time
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot(t(idx_burn),  alt_plot(idx_burn),  'r-', 'LineWidth', 1.5);
plot(t(idx_coast), alt_plot(idx_coast), 'k-', 'LineWidth', 1.5);

plot(t(idx_altmax), alt_max, 'ko', 'MarkerFaceColor', [0.2 0.2 0.1], 'MarkerSize', 6);
text(t(idx_altmax), alt_max, sprintf('  X %.3f\n  Y %.5f', t(idx_altmax), alt_max), ...
    'FontSize', 11, ...
    'Color', [0 0.45 0.8], ...
    'BackgroundColor', [0.93 0.93 0.96], ...
    'Margin', 8);

xlabel('Time [sec]', 'FontSize', 14);
ylabel('Altitude [m]', 'FontSize', 14);
legend('burning', 'coasting', 'Location', 'northeast');
set(gca, 'FontSize', 13);

%% =========================================================
% Figure 3. Altitude vs Downrange
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot(downrange(idx_burn),  alt_plot(idx_burn),  'r-', 'LineWidth', 1.5);
plot(downrange(idx_coast), alt_plot(idx_coast), 'k-', 'LineWidth', 1.5);

plot(downrange(idx_altmax), alt_max, 'ko', 'MarkerFaceColor', [0.2 0.2 0.1], 'MarkerSize', 6);
text(downrange(idx_altmax), alt_max, ...
    sprintf('  X %.2f\n  Y %.3f', downrange(idx_altmax), alt_max), ...
    'FontSize', 11, ...
    'Color', [0 0.45 0.8], ...
    'BackgroundColor', [0.93 0.93 0.96], ...
    'Margin', 8);

xlabel('Downrange [m]', 'FontSize', 14);
ylabel('Altitude [m]', 'FontSize', 14);
legend('burning', 'coasting', 'Location', 'northeast');
set(gca, 'FontSize', 13);

%% =========================================================
% Figure 4. Airspeed vs Time
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot(t(idx_burn),  V_rel(idx_burn,1),  'r-',  'LineWidth', 1.5);
plot(t(idx_burn),  V_rel(idx_burn,2),  'r-.', 'LineWidth', 1.2);
plot(t(idx_burn),  V_rel(idx_burn,3),  'r:',  'LineWidth', 1.2);

plot(t(idx_coast), V_rel(idx_coast,1), 'k-',  'LineWidth', 1.5);
plot(t(idx_coast), V_rel(idx_coast,2), 'k-.', 'LineWidth', 1.2);
plot(t(idx_coast), V_rel(idx_coast,3), 'k:',  'LineWidth', 1.2);

[Vxmax, idx_vxmax] = max(V_rel(:,1));
plot(t(idx_vxmax), Vxmax, 'ko', 'MarkerFaceColor', [0.2 0.2 0.1], 'MarkerSize', 6);
text(t(idx_vxmax), Vxmax, sprintf('  X %.3f\n  Y %.4f', t(idx_vxmax), Vxmax), ...
    'FontSize', 11, ...
    'Color', [0 0.45 0.8], ...
    'BackgroundColor', [0.93 0.93 0.96], ...
    'Margin', 8);

xlabel('Time [sec]', 'FontSize', 14);
ylabel('Body Velocity [m/s]', 'FontSize', 14);
legend('v_x burning','v_y burning','v_z burning', ...
       'v_x coasting','v_y coasting','v_z coasting', ...
       'Location', 'northeast');
set(gca, 'FontSize', 13);

%% =========================================================
% Figure 5. Body Axial Acceleration vs Time
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot(t(idx_burn),  ax_g(idx_burn),  'r-',  'LineWidth', 1.5);
plot(t(idx_burn),  ay_g(idx_burn),  'r-.', 'LineWidth', 1.2);
plot(t(idx_burn),  az_g(idx_burn),  'r:',  'LineWidth', 1.2);

plot(t(idx_coast), ax_g(idx_coast), 'k-',  'LineWidth', 1.5);
plot(t(idx_coast), ay_g(idx_coast), 'k-.', 'LineWidth', 1.2);
plot(t(idx_coast), az_g(idx_coast), 'k:',  'LineWidth', 1.2);

plot(t(idx_axmax), axmax_g, 'ko', 'MarkerFaceColor', [0.2 0.2 0.1], 'MarkerSize', 6);
text(t(idx_axmax), axmax_g, sprintf('  X %.3f\n  Y %.5f', t(idx_axmax), axmax_g), ...
    'FontSize', 11, ...
    'Color', [0 0.45 0.8], ...
    'BackgroundColor', [0.93 0.93 0.96], ...
    'Margin', 8);

xlabel('Time [sec]', 'FontSize', 14);
ylabel('Body Acceleration [G]', 'FontSize', 14);
legend('a_x burning','a_y burning','a_z burning', ...
       'a_x coasting','a_y coasting','a_z coasting', ...
       'Location', 'northeast');
set(gca, 'FontSize', 13);

%% =========================================================
% Figure 6. 3D Flight Trajectory
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot3(x(idx_burn),  y(idx_burn),  alt_plot(idx_burn),  'r-', 'LineWidth', 1.5);
plot3(x(idx_coast), y(idx_coast), alt_plot(idx_coast), 'k-', 'LineWidth', 1.5);

xlabel('x [m]', 'FontSize', 14);
ylabel('y [m]', 'FontSize', 14);
zlabel('z [m]', 'FontSize', 14);
legend('burning', 'coasting', 'Location', 'northeast');
set(gca, 'FontSize', 13);
view(35, 22);

%% =========================================================
% Figure 7. 3D Flight Trajectory & 6-DOF Pose Animation (NED 정석)
%% =========================================================
disp('🚀 3D 궤적 및 자세 통합 애니메이션을 준비합니다...');

% 1) 오일러 각 데이터 불러오기
try
    Euler_angles = localToMatrix(simOut.Euler_angles);
    Euler_angles = Euler_angles(1:N, :);
    euler_rad = deg2rad(Euler_angles);
catch
    error('simOut에 Euler_angles 데이터가 없습니다.');
end

% 2) 비디오 저장 설정
video_filename = 'Rocket_Trajectory_Pose_Corrected.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30;
v.Quality = 100;
open(v);

% 3) Figure 초기화 및 직교성 강제
target_width = 1280;
target_height = 720;
fig = figure('Name', '3D Trajectory & Pose (Right-Handed)', 'Color', 'w');
set(fig, 'Position', [100, 100, target_width, target_height]);
hold on; grid on; box on;

% 🚨 [핵심 해결책 1] 데이터 종횡비 1:1:1 강제 (축 찌그러짐 방지)
daspect([1 1 1]); 

% 🚨 [핵심 해결책 2] 축 반전 설정
% ZDir을 reverse하여 Down이 아래를 향하게 하고, YDir을 reverse하여 오른손 법칙 시점을 유지합니다.
set(gca, 'ZDir', 'reverse');
set(gca, 'YDir', 'reverse');
view(45, 20);

% 4) 🌟 [요구사항 반영] 3D 좌표계 원점(0,0,0)에 기준 좌표계(Earth Frame) 표시
ref_len = max(max(x)-min(x), max(abs(z_E))) * 0.1; % 궤적 크기에 맞춰 기준축 길이 설정
if ref_len < 10, ref_len = 10; end

plot3([0 ref_len], [0 0], [0 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
plot3([0 0], [0 ref_len], [0 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
plot3([0 0], [0 0], [0 ref_len], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);

text(ref_len*1.1, 0, 0, '$X_E$ (North)', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
text(0, ref_len*1.1, 0, '$Y_E$ (East)', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');
text(0, 0, ref_len*1.1, '$Z_E$ (Down)', 'Interpreter', 'latex', 'FontSize', 12, 'FontWeight', 'bold');

% 축 범위 고정 (z_E는 음수/양수 모두 포함되므로 범위를 넉넉히 잡습니다)
margin_x = (max(x) - min(x)) * 0.1 + 10;
margin_y = (max(y) - min(y)) * 0.1 + 10;
margin_z = (max(abs(z_E))) * 0.1 + 10;

xlim([min(x)-margin_x, max(x)+margin_x]);
ylim([min(y)-margin_y, max(y)+margin_y]);
zlim([min(z_E)-margin_z, max(z_E)+margin_z]);

xlabel('North [m]', 'FontSize', 12);
ylabel('East [m]', 'FontSize', 12);
zlabel('Down [m]', 'FontSize', 12);

% 5) 궤적 및 기체 객체 초기화
% 🚨 [핵심 해결책 3] alt_plot 대신 순수한 z_E 사용
hTraj_burn  = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 2);
hTraj_coast = plot3(NaN, NaN, NaN, 'k-', 'LineWidth', 2);

axis_len = ref_len * 0.5; % 기체 축의 길이 (기준 좌표계의 절반 크기)
pX = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 3); 
pY = plot3([0 0], [0 0], [0 0], 'g', 'LineWidth', 3);
pZ = plot3([0 0], [0 0], [0 0], 'b', 'LineWidth', 3);

label_dist = axis_len * 1.2;
hText_XB = text(0, 0, 0, '$X_B$', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');
hText_YB = text(0, 0, 0, '$Y_B$', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'g');
hText_ZB = text(0, 0, 0, '$Z_B$', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'b');

hPos = plot3(x(1), y(1), z_E(1), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');

% 6) 애니메이션 루프
target_frames = 400; 
skip_step = max(1, round(N / target_frames));

for i = 1:skip_step:N
    % (A) 궤적 업데이트 (순수 z_E 사용)
    if i <= burn_end_idx
        set(hTraj_burn, 'XData', x(1:i), 'YData', y(1:i), 'ZData', z_E(1:i));
    else
        set(hTraj_burn, 'XData', x(1:burn_end_idx), 'YData', y(1:burn_end_idx), 'ZData', z_E(1:burn_end_idx));
        set(hTraj_coast, 'XData', x(burn_end_idx:i), 'YData', y(burn_end_idx:i), 'ZData', z_E(burn_end_idx:i));
    end
    
    cx = x(i); cy = y(i); cz = z_E(i);
    set(hPos, 'XData', cx, 'YData', cy, 'ZData', cz);
    
    % (B) 기체 자세 회전 (NED 기준)
    C_b2e = eul2rotm(euler_rad(i, :), 'ZYX');
    e1 = [1; 0; 0]; e2 = [0; 1; 0]; e3 = [0; 0; 1];
    
    % 🚨 Z성분 부호 반전 없이 순수 계산값 그대로 사용 (오른손 법칙 완벽 유지)
    X_vec = C_b2e * e1 * axis_len;
    Y_vec = C_b2e * e2 * axis_len;
    Z_vec = C_b2e * e3 * axis_len;
    
    % (C) 기체 축 그리기
    set(pX, 'XData', [cx, cx + X_vec(1)], 'YData', [cy, cy + X_vec(2)], 'ZData', [cz, cz + X_vec(3)]);
    set(pY, 'XData', [cx, cx + Y_vec(1)], 'YData', [cy, cy + Y_vec(2)], 'ZData', [cz, cz + Y_vec(3)]);
    set(pZ, 'XData', [cx, cx + Z_vec(1)], 'YData', [cy, cy + Z_vec(2)], 'ZData', [cz, cz + Z_vec(3)]);
    
    % (D) 라벨 위치 업데이트
    Lbl_X = C_b2e * e1 * label_dist;
    Lbl_Y = C_b2e * e2 * label_dist;
    Lbl_Z = C_b2e * e3 * label_dist;
    
    set(hText_XB, 'Position', [cx + Lbl_X(1), cy + Lbl_X(2), cz + Lbl_X(3)]);
    set(hText_YB, 'Position', [cx + Lbl_Y(1), cy + Lbl_Y(2), cz + Lbl_Y(3)]);
    set(hText_ZB, 'Position', [cx + Lbl_Z(1), cy + Lbl_Z(2), cz + Lbl_Z(3)]);
    
    % 제목 업데이트 (z_E는 아래가 양수이므로 -z_E를 출력하여 고도로 표시)
    title(sprintf('Flight Trajectory & Pose (Right-Handed) | Time: %.2f s\nAlt: %.1f m | Mach: %.2f', ...
          t(i), -cz, norm(v_B(i,:))/340));
    
    drawnow;
    
    % (E) 프레임 캡처
    frame = getframe(fig);
    img = imresize(frame.cdata, [target_height, target_width]);
    writeVideo(v, img);
end

close(v);
disp('✅ 완벽한 직교 좌표계를 갖춘 통합 애니메이션 비디오 저장이 완료되었습니다.');
%% =========================================================
% Figure 7. Landing / Splash range
%% =========================================================
figure('Color','w');
hold on; grid on; box on;

plot(x, y, 'b-', 'LineWidth', 1.3);
plot(0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(landing_x, landing_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

text(landing_x, landing_y, ...
    sprintf(' Landing\n x = %.2f m\n y = %.2f m\n R = %.2f m', ...
    landing_x, landing_y, landing_downrange), ...
    'FontSize', 11, ...
    'BackgroundColor', [0.95 0.95 0.95], ...
    'Margin', 8);

xlabel('x [m]', 'FontSize', 14);
ylabel('y [m]', 'FontSize', 14);
title('Landing / Splash point', 'FontSize', 15);
axis equal;
set(gca, 'FontSize', 13);

%% 10) 결과 출력
fprintf('\n========== Postprocess Result ==========\n');
fprintf('Burn end time         : %.4f s\n', burn_end_time);
fprintf('Max dynamic pressure  : %.4f kPa at t = %.4f s\n', qmax, t(idx_qmax));
fprintf('Max altitude          : %.4f m at t = %.4f s\n', alt_max, t(idx_altmax));
fprintf('Max axial accel       : %.4f G at t = %.4f s\n', axmax_g, t(idx_axmax));
fprintf('Landing time          : %.4f s\n', landing_time);
fprintf('Landing point         : x = %.4f m, y = %.4f m\n', landing_x, landing_y);
fprintf('Landing downrange     : %.4f m\n', landing_downrange);
fprintf('CSV saved             : landing_splash_point.csv\n');
fprintf('========================================\n');

%% =========================================================
% local functions
%% =========================================================
function out = localToColumn(sig)

    if isa(sig, 'timeseries')
        out = sig.Data;

    elseif isa(sig, 'Simulink.SimulationData.Signal')
        out = sig.Values.Data;

    elseif isstruct(sig)
        if isfield(sig, 'signals') && isfield(sig.signals, 'values')
            out = sig.signals.values;
        elseif isfield(sig, 'Data')
            out = sig.Data;
        else
            error('localToColumn: struct에서 데이터를 찾을 수 없습니다.');
        end

    elseif isnumeric(sig)
        out = sig;

    else
        error('localToColumn: 지원하지 않는 타입입니다. class = %s', class(sig));
    end

    out = squeeze(out);

    if ~isnumeric(out)
        error('localToColumn: 변환 후 숫자형이 아닙니다. class = %s', class(out));
    end

    out = out(:);
end

function out = localToMatrix(sig)

    if isa(sig, 'timeseries')
        out = sig.Data;

    elseif isa(sig, 'Simulink.SimulationData.Signal')
        out = sig.Values.Data;

    elseif isstruct(sig)
        if isfield(sig, 'signals') && isfield(sig.signals, 'values')
            out = sig.signals.values;
        elseif isfield(sig, 'Data')
            out = sig.Data;
        else
            error('localToMatrix: struct에서 데이터를 찾을 수 없습니다.');
        end

    elseif isnumeric(sig)
        out = sig;

    else
        error('localToMatrix: 지원하지 않는 타입입니다. class = %s', class(sig));
    end

    out = squeeze(out);

    if ~isnumeric(out)
        error('localToMatrix: 변환 후 숫자형이 아닙니다. class = %s', class(out));
    end

    if isvector(out)
        out = out(:);
        return;
    end

    if ndims(out) ~= 2
        error('localToMatrix: squeeze 후에도 2차원이 아닙니다. size = %s', mat2str(size(out)));
    end

    [r, c] = size(out);

    if r <= 3 && c > 3
        out = out.';
    end

    
end