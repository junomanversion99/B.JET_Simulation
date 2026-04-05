function [F_aero_B, M_aero_B] = Calculate_F_aero_B_and_M_aero_B(Q, V_mag, Coeffs, CG_current, w_B, S_ref, D_ref)
    %% 입력 정의:
    % Q       : 동압 (Dynamic Pressure) [N/m^2] 
    % V_mag   : 속도 크기 (Velocity Magnitude) [m/s]
    % w_B     : 각속도 벡터 [p; q; r] [rad/s] 3x1
    % Coeffs  : Bus Creator로부터 온 9개 공력 계수 구조체 (C_A, C_Y, C_Z, C_LL, C_M, C_LN, C_LLp, C_Mq, C_LNr)
    % S_ref   : 기준 면적 (Reference Area) [m^2]
    % D_ref   : 기준 직경 (Reference Diameter) [m]
    % CG_current    : 

    %% 출력 정의:
    % F_aero_B : 기체 좌표계 공력 힘 [F_x; F_y; F_z] [N] 3x1
    % M_aero_B : 기체 좌표계 공력 모멘트 [M_l; M_m; M_n] [N*m] 3x1
    
    %% 출력 크기 명시
    F_aero_B = zeros(3,1);
    M_aero_B = zeros(3,1);

    %% 1. 각속도 성분 추출 [rad/s]
    p = w_B(1);
    q = w_B(2);
    r = w_B(3);

    %% 2. 수치적 안정성을 위한 속도 임계값 설정
    % 속도가 0인 초기 상태에서 분모가 0이 되어 NaN이 발생하는 것 방지
    eps_v = 5;
    if V_mag < eps_v
        v_denom = eps_v; 
    else
        v_denom = V_mag;
    end


    %% DATCOM 공력좌표계기준 공력계수를 기체좌표계 기준 공력계수로 변환
    C_X = -Coeffs.C_A;
    C_Y = Coeffs.C_Y;
    C_Z = -Coeffs.C_N;

    C_L = Coeffs.C_LL;
    C_M = Coeffs.C_M;
    C_N = Coeffs.C_LN;

    C_Lp = Coeffs.C_LLp;
    C_Mq = Coeffs.C_Mq;
    C_Nr = Coeffs.C_LNr;
 
    %% 3. 공력 힘(Aerodynamic Forces) 계산
    % 수식: F = Q * S_ref * [C_X; C_Y; C_Z]
    

    F_x = Q * S_ref * C_X;
    F_y = Q * S_ref * C_Y;
    F_z = Q * S_ref * C_Z;

    F_aero_B = [F_x; F_y; F_z];

    %% 4. 공력 모멘트(Aerodynamic Moments) 계산
    % 수식: M = Q * S_ref * D_ref * [C_l + (D/2v)*C_Lp*p; C_m + (D/2v)*C_Mq*q; C_n + (D/2v)*C_Nr*r]
    
    % dX_non_dim: 기준점으로부터 현재 무게중심까지의 거리 차이 무차원화
    %  -> 이거 없으면 9초~14초 부근에서 CG가 CP뒤로 넘어가면서 뒤집힘

    % X_cg  : 현재 비행 시점의 무게중심 위치 (기수로부터의 거리, [m])
    % 1.2(=X_ref) : Missile Datcom에 입력했던 모멘트 기준점 (X_CG값임)
    % Prop.CG_initial = 1.18768; 
    % dX    : 기준점으로부터 현재 무게중심까지의 거리 차이 무차원화
    
    dX_non_dim = (CG_current - 1.18768) / D_ref; 
    

    % Datcom 기준 피칭 모멘트에 C.G. 이동으로 인해 추가되는 모멘트를 더함
    % C_Z가 양수(기체 아래로 향하는 힘)일 때 C.G.가 뒤로 가면(dX > 0) 기수가 들리는(+) 모멘트 발생   
    C_M_CG = Coeffs.C_M + (Coeffs.C_N * dX_non_dim);
    C_N_CG = Coeffs.C_LN + (Coeffs.C_Y * dX_non_dim);
    
    % 댐핑 팩터 (D/2v) 계산
    damping_factor = D_ref / (2 * v_denom);

    % 최종 모멘트 계산 (보정된 정적 계수 + 댐핑 계수)
    M_l = Q * S_ref * D_ref * (C_L + damping_factor * C_Lp * p); % 기체가 좌우 대칭이면 순수 롤 멘트는 크게 생기지 않음 -> 0으로 해도됨
    M_m = Q * S_ref * D_ref * (C_M_CG + damping_factor * C_Mq * q);
    M_n = Q * S_ref * D_ref * (C_N_CG + damping_factor * C_Nr * r); 

    M_aero_B = [M_l; M_m; M_n];


end
