% [kg]
Mass.mass_initial = 14;
Mass.mass_fuel = 1.57;
Mass.mass_final = Mass.mass_initial - Mass.mass_fuel;
%{
% [kg * m^2]
Mass.MOI_initial = [0.036, 0, 0;
                    0, 2.886, 0;
                    0, 0, 2.886];

Mass.MOI_final   = [0.034, 0, 0;
                    0, 2.731, 0;
                    0, 0, 2.731];
%}
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
Prop.burn_time = 7.3; % [s] 예시 연소 시간
Prop.mass_dot_negative = (Mass.mass_initial - Mass.mass_final) / Prop.burn_time;
Prop.MOI_dot_negative = (Mass.MOI_initial - Mass.MOI_final) / Prop.burn_time;

% CG 변화율 (Nose로부터의 거리 [m] 예시)
Prop.CG_initial = 1.18768; 
Prop.CG_final = 1.17257;
Prop.CG_dot_negative = (Prop.CG_initial - Prop.CG_final) / Prop.burn_time;


%Prop.Thrust = 0;                                                 % 추후 값 받기

Geom.D_ref = 0.113; % [m]
Geom.S_ref = (Geom.D_ref^2)*(pi/4); % [m^2]

Init.launch_angle = [0; 85; 0];                                       % [deg] 낙하범위 계산할 때 조절하세용 (초반 회전행렬값도 바꿔야함)
Init.initial_launch_angle = deg2rad(Init.launch_angle);       % launch angle[deg]을 [rad]로 바꿔 입력

%% 초기 자세 DCM 생성 (Integrator_C 초기조건용)
% launch_angle = [yaw; pitch; roll] [deg] 
psi0   = deg2rad(Init.launch_angle(1)); % yaw
theta0 = deg2rad(Init.launch_angle(2)); % pitch
phi0   = deg2rad(Init.launch_angle(3)); % roll

% Body -> Earth DCM (ZYX: yaw-pitch-roll)
cpsi = cos(psi0);   spsi = sin(psi0);
cth  = cos(theta0); sth  = sin(theta0);
cphi = cos(phi0);   sphi = sin(phi0);

C_b2e = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
          spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
          -sth,      cth*sphi,                 cth*cphi ];

% Earth -> Body DCM (Integrator에서 C_e2b를 상태로 적분한다는 가정)
Init.C0_e2b = C_b2e.';

Init.escape_altitude = 5 * sin(deg2rad(Init.launch_angle(2)));            % 5는 launch pad length[m]

Init.latitude_initial = 40.138633;                                               % 발사지점 위도 [deg]
Init.logitutde_initial = 139.984850;                                             % 발사지점 경도 [deg]

%% 카나드 있는 공력계수 데이터 받아오기
AeroDB = load('Aero_DB_Roll_Canard_On_0320.mat');

Aero.delrVec  = AeroDB.delrVec(:)';
Aero.alphaVec = AeroDB.alphaVec(:)';
Aero.betaVec  = AeroDB.betaVec(:)';
Aero.machVec  = AeroDB.machVec(:)';
Aero.altVec   = AeroDB.altVec(:)';
Aero.altVec_dynamic = Aero.altVec;

Aero.CA_DB   = AeroDB.CA_DelR;
Aero.CY_DB   = AeroDB.CY_DelR;
Aero.CN_DB   = AeroDB.CN_DelR;
Aero.CLL_DB  = AeroDB.CLL_DelR;
Aero.CM_DB   = AeroDB.CM_DelR;
Aero.CLN_DB  = AeroDB.CLN_DelR;

Aero.CLLP_DB = AeroDB.CLLP_DelR;
Aero.CMQ_DB  = AeroDB.CMQ_DelR;
Aero.CLNR_DB = AeroDB.CLNR_DelR;

%% 카나드 없는 공력계수 데이터 받아오기
%{
AeroDB = load('Aero_DB_Roll.mat');

Aero.alphaVec = AeroDB.alphaVec(:)';
Aero.betaVec  = AeroDB.betaVec(:)';
Aero.machVec  = AeroDB.machVec(:)';
Aero.altVec   = AeroDB.altVec(:)';
Aero.altVec_dynamic = AeroDB.altVec_dynamic(:)';

Aero.CA_DB   = AeroDB.CA_DB;
Aero.CY_DB   = AeroDB.CY_DB;
Aero.CN_DB   = AeroDB.CN_DB;
Aero.CLL_DB  = AeroDB.CLL_DB;
Aero.CM_DB   = AeroDB.CM_DB;
Aero.CLN_DB  = AeroDB.CLN_DB;

Aero.CLLP_DB = AeroDB.CLLP_DB;
Aero.CMQ_DB  = AeroDB.CMQ_DB;
Aero.CLNR_DB = AeroDB.CLNR_DB;
%}

%% 추력 데이터 받아오기
ThrustTbl = readtable('Thrust_Raw_Data_polaris.csv');

Prop.t_thrust = ThrustTbl{:,1};
Prop.F_thrust = ThrustTbl{:,2};

%% 버스 오류 해결
names = {'C_A','C_Y','C_N','C_LL','C_LLp','C_M','C_Mq','C_LN','C_LNr'};

clear elems
for k = 1:numel(names)
    elems(k) = Simulink.BusElement;
    elems(k).Name = names{k};
    elems(k).Dimensions = 1;
    elems(k).DataType = 'double';
end

AeroCoeffBus = Simulink.Bus;
AeroCoeffBus.Elements = elems;

%% Date 정의
Date0 = 2460000;