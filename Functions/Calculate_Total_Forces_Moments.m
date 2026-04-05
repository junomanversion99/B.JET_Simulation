%{
function [F_total_B,M_total_B] = Calculate_Total_Forces_Moments(F_aero_B, M_aero_B, CG_current, mass_current, F_thrust, escape_trigger, g_Body)
    % 입력 정의:
    % F_aero_B      : [F_x; F_y; F_z] 공력 [N]
    % M_aero_B      : [M_l; M_m; M_n] 공력 모멘트 [N*m]
    % F_thrust_B    : [Thrust_raw; 0; 0] 추력 벡터 [N]
    % mass_current  : 현재 질량 [kg]
    % CG_current    : 노즈로부터 현재 무게중심까지의 거리 [m]
    % escape_trigger: 발사대 이탈 여부 (0: 레일 위, 1: 자유비행)
    % r_MRP         : 초기 무게중심 위치 (Nose 기준 거리) [m] (Parameter)
    % g_Body        : 기체좌표계로 변환된 중력가속도 벡터 [m/s^2]
    r_MRP = 1.2;
    %% 1. 중력 벡터 계산 (Body Frame)
    % 입력받은 g_body를 질량과 곱하여 중력 산출
    F_weight_B = mass_current * g_Body;

    %% 2. 총 외력 합산 (Total Force)
    
    F_total_B = F_thrust_B + F_aero_B + F_weight_B;

    %% 3. 총 모멘트 합산 (Total Moment)
    % M_total_B = M_aero_B + (r_MRP - r_BG) x F_aero_B
    % 노즈로부터의 거리 차이를 X축 벡터로 변환
    
    dist_offset = r_MRP - CG_current;
    r_offset_vec = [dist_offset; 0; 0];
    
    % 외적(Cross Product)을 통한 모멘트 보정항 계산
    M_offset_B = cross(r_offset_vec, F_aero_B);
    
    M_total_B = M_aero_B + M_offset_B;

    %% 4. 발사대 구속 로직 (Launch Rail Constraint)
    % escape_trigger가 0(레일 위)일 경우:
    % - 로켓은 X축 방향으로만 움직일 수 있음 (Y, Z 힘 제거)
    % - 로켓은 회전할 수 없음 (모든 모멘트 제거)
    if escape_trigger == 0
        F_total_B(2:3) = 0; % Fy, Fz 제거
        M_total_B(:)   = 0; % Ml, Mm, Mn 제거
    end
end
%}

function [F_total_B,M_total_B] = Calculate_Total_Forces_Moments(F_aero_B, M_aero_B, CG_current, mass_current, F_thrust_B, escape_trigger, g_Body)

    % 입력 정의:
    % F_aero_B      : [F_x; F_y; F_z] 공력 [N]
    % M_aero_B      : [M_l; M_m; M_n] 공력 모멘트 [N*m]
    % F_thrust      : [Thrust;0;0] 벡터 [N]
    % mass_current  : 현재 질량 [kg]
    % CG_current    : 노즈로부터 현재 무게중심까지의 거리 [m]
    % escape_trigger: 발사대 이탈 여부 (0: 레일 위, 1: 자유비행)
    % g_Body        : 기체좌표계 중력가속도 벡터 [m/s^2]

    %% 출력 크기 미리 지정
    F_total_B = zeros(3,1);
    M_total_B = zeros(3,1);

    %% 기준점(MRP) 위치 [m]
    r_MRP = 1.2;

    %% 입력 형상 강제
    F_aero_B = reshape(F_aero_B, [3,1]);
    M_aero_B = reshape(M_aero_B, [3,1]);
    g_Body   = reshape(g_Body,   [3,1]);


    %% 1. 중력 벡터 계산 (Body Frame)
    F_weight_B = mass_current * g_Body;

    %% 2. 총 외력 합산
    F_total_B = F_thrust_B + F_aero_B + F_weight_B;

    %% 3. 총 모멘트 합산
    % M_total_B = M_aero_B + (r_MRP - CG_current) x F_aero_B

    dist_offset = r_MRP - CG_current;
    r_offset_vec = [dist_offset; 0; 0];
    r_offset_vec = reshape(r_offset_vec, [3,1]);

    M_offset_B = cross(r_offset_vec, F_aero_B);
    M_total_B = M_aero_B + M_offset_B;

    %% 4. 발사대 구속 로직
    if escape_trigger == 0
        F_total_B(2:3) = 0;
        M_total_B(:)   = 0;
    end
end