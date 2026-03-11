% [kg]
Mass.mass_initial = 13;
Mass.mass_fuel = 1.303;
Mass.mass_final = Mass.mass_initial - Mass.mass_fuel;

% [kg * mm^2]
Mass.MOI_initial_raw = [39863.025,  -383.345,    2309.809;
                   -383.345,   5131347.368, 3.468;
                   2309.809,   3.468,       5132752.234];

Mass.MOI_final_raw = [38876.733,  -384.646,    2305.200;
                 -384.646,   5121620.512, 3.470;
                 2305.200,   3.470,       5123025.382];

% [kg * m^2]
Mass.MOI_initial = Mass.MOI_initial_raw * 10^-6;
Mass.MOI_final = Mass.MOI_final_raw * 10^-6;

% Set_parameters.m 하단에 추가
Prop.burn_time = 2.5; % [s] 예시 연소 시간
Prop.mass_dot_negative = (Mass.mass_initial - Mass.mass_final) / Prop.burn_time;
Prop.MOI_dot_negative = (Mass.MOI_initial - Mass.MOI_final) / Prop.burn_time;

% CG 변화율 (Nose로부터의 거리 [m] 예시)
Prop.CG_initial = 1.2; 
Prop.CG_final = 1.15;
Prop.CG_dot_negative = (Prop.CG_initial - Prop.CG_final) / Prop.burn_time;


Prop.Thrust = 0;                                                 % 추후 값 받기
% [m]
Geom.D_ref = 0.113;
Geom.S_ref = (Geom.D_ref^2)*(pi/4);

Init.launch_angle = [0; 80; 0];                                  % [deg] 낙하범위 계산할 때 조절하세용
Init.initial_launch_angle = [0; deg2rad(Init.launch_angle); 0];       % launch angle[deg]을 [rad]로 바꿔 입력
Init.escape_altitude = sin(deg2rad(Init.launch_angle))*5;             % 5는 launch pad length[m]

Init.latitude_initial = 40.138633;                                               % 발사지점 위도 [deg]
Init.logitutde_initial = 139.984850;                                             % 발사지점 경도 [deg]