function [rho, V_sound] = Calculate_V_sound_and_rho(z_E, g_Body)

%% 입력: z_E [m], g_Body [m/s^2]
%% 출력: rho [kg/m^3], V_sound [m/s]

%% 상수값 정의 국제표준대기 참고
T0    = 288.15;     % [K]
p0    = 101325.0;   % [Pa]
%g0    = 9.80665;    % [m/s^2]
g = norm(g_Body);
R     = 287.05;     % [J/(kg·K)]
k     = 1.4;        % [-]
L     = 0.0065;     % [K/m] lapse rate (0~11 km)

%% 고도 상한, 하한선 설정 (대류권 구간)
if z_E < 0
    z_E = 0;
elseif z_E > 11000
    z_E = 11000;
end

%% 대류권 내 계산 모델 
T = T0 - L*z_E;                   % 온도 [K]
p = p0 * (T/T0)^(g/(R*L));       % 압력 [Pa]

%% 출력
rho = p/(R*T);                % 밀도 [kg/m^3]
V_sound = sqrt(k*R*T);        % 음속 [m/s]
end