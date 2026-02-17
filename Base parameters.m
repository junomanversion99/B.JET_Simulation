% [kg]
mass_initial = 13;
mass_fuel = 1.303;
mass_final = mass_initial - mass_fuel;

% [kg * mm^2]
MOI_initial_raw = [39863.025,  -383.345,    2309.809;
                   -383.345,   5131347.368, 3.468;
                   2309.809,   3.468,       5132752.234];

MOI_final_raw = [38876.733,  -384.646,    2305.200;
                 -384.646,   5121620.512, 3.470;
                 2305.200,   3.470,       5123025.382];

% [kg * m^2]
MOI_initial = MOI_initial_raw * 10^-6;
MOI_final = MOI_final_raw * 10^-6;

Thrust = 0;                                                 % 추후 값 받기
m_dot_negative = 0;                                         % m dot 값에 -1을 곱해서 추후 계산에 용이하도록

% [m]
D_ref = 0.113;
S_ref = (D_ref^2)*(pi/4);

launch_angle = [0; 80; 0];                                  % [deg] 낙하범위 계산할 때 조절하세용
initial_launch_angle = [0; deg2rad(launch_angle); 0];       % launch angle[deg]을 [rad]로 바꿔 입력
escape_altitude = sin(deg2rad(launch_angle))*5;             % 5는 launch pad length[m]