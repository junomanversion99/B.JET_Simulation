function [F_aero_B, M_aero_B] = Calculate_F_aero_B_and_M_aero_B(Q, V_mag, w_B, Coeffs, S_ref, D_ref)
    % 입력 정의:
    % Q       : 동압 (Dynamic Pressure) [N/m^2]
    % V_mag   : 속도 크기 (Velocity Magnitude) [m/s]
    % w_B     : 각속도 벡터 [p; q; r] [rad/s]
    % Coeffs  : Bus Creator로부터 온 9개 공력 계수 구조체 (C_x, C_y, C_z, C_l, C_m, C_n, C_Lp, C_Mq, C_Nr)
    % S_ref   : 기준 면적 (Reference Area) [m^2]
    % D_ref   : 기준 직경 (Reference Diameter) [m]

    % 출력 정의:
    % F_aero_B : 기체 좌표계 공력 힘 [Fx; Fy; Fz] [N]
    % M_aero_B : 기체 좌표계 공력 모멘트 [Ml; Mm; Mn] [N*m]

    %% 1. 각속도 성분 추출 [rad/s]
    p = w_B(1);
    q = w_B(2);
    r = w_B(3);

    %% 2. 수치적 안정성을 위한 속도 임계값 설정
    % 속도가 0인 초기 상태에서 분모가 0이 되어 NaN이 발생하는 것 방지
    eps_v = 1e-6;
    if V_mag < eps_v
        v_denom = eps_v; 
    else
        v_denom = V_mag;
    end

    %% 3. 공력 힘(Aerodynamic Forces) 계산
    % 수식: F = Q * S_ref * [C_x; C_y; C_z]
    F_x = Q * S_ref * Coeffs.C_x;
    F_y = Q * S_ref * Coeffs.C_y;
    F_z = Q * S_ref * Coeffs.C_z;

    F_aero_B = [F_x; F_y; F_z];

    %% 4. 공력 모멘트(Aerodynamic Moments) 계산
    % 수식: M = Q * S_ref * D_ref * [C_l + (D/2v)*C_Lp*p; C_m + (D/2v)*C_Mq*q; C_n + (D/2v)*C_Nr*r]
    
    % 댐핑 팩터 (D/2v) 계산
    damping_factor = D_ref / (2 * v_denom);

    M_l = Q * S_ref * D_ref * (Coeffs.C_l + damping_factor * Coeffs.C_Lp * p);
    M_m = Q * S_ref * D_ref * (Coeffs.C_m + damping_factor * Coeffs.C_Mq * q);
    M_n = Q * S_ref * D_ref * (Coeffs.C_n + damping_factor * Coeffs.C_Nr * r);

    M_aero_B = [M_l; M_m; M_n];

end