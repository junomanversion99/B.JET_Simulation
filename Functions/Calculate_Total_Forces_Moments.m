function [F_total_B,M_total_B] = Calculate_Total_Forces_Moments(mass_current, g_body, F_aero_B, M_aero_B, CG_current, F_thrust, escape_trigger)
    % 입력 정의:
    % F_aero_B      : [Fx; Fy; Fz] 공력 [N]
    % M_aero_B      : [Ml; Mm; Mn] 공력 모멘트 [N*m]
    % F_thrust_B    : [Thrust_raw; 0; 0] 추력 벡터 [N]
    % mass_current  : 현재 질량 [kg]
    % CG_current    : 노즈로부터 현재 무게중심까지의 거리 [m]
    % escape_trigger: 발사대 이탈 여부 (0: 레일 위, 1: 자유비행)
    % r_MRP         : 초기 무게중심 위치 (Nose 기준 거리) [m] (Parameter)
    % g_body        : 기체좌표계로 변환된 중력가속도 벡터 [m/s^2]

    %% 1. 중력 벡터 계산 (Body Frame)
    % 입력받은 g_body를 질량과 곱하여 중력 산출
    F_weight_B = mass_current * g_body;

    %% 2. 총 외력 합산 (Total Force)
    
    F_total_B = F_thrust + F_aero_B + F_weight_B;

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
