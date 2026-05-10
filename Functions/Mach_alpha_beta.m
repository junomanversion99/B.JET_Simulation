function [Mach, alpha, beta] = Mach_alpha_beta(V_rel, V_sound)
    %% 상대속도 V_rel 과 음속 V_sound 를 이용해서 마하수, 받음각, 옆미끄럼각 계산

    %% 입력
    % V_rel = [u_rel; v_rel; w_rel]   : 바디좌표계에 투영한 로켓의 대기에 대한 상대속도 [m/s],  3x1
    % V_sound                         : 음속 [m/s]

    %% 출력
    % Mach                            : 마하수 [무차원수]
    % alpha                           : 받음각 [rad] (주의: Lookup Table 입력 전 외부에서 deg 변환)
    % beta                            : 옆미끄럼각 [rad]
    
    % V_rel = [u_rel; v_rel; w_rel]
    u_rel = V_rel(1);
    v_rel = V_rel(2);
    w_rel = V_rel(3);
    
    % 
    V_mag = norm(V_rel);

    
    % --- 예외 처리 시작 ---
    eps_v = 1.0; % 속도 임계값 (m/s) -> 1e-6은 너무 작다... 저속특이점 현상 생김
    
    if V_mag < eps_v
        % 공력이 유의미하지 않은 저속 구간에서는 각도를 0으로 묶음
        Mach = 0;
        alpha = 0;
        beta = 0;
    else
        % 일반적인 비행 상태
        
        % 1. 마하수 계산 (V_sound가 0이 들어오는 시스템 오류 대비 방어)
        Mach = V_mag / max(V_sound, 1e-6);
        
       % 2. 받음각 (alpha) 계산 [rad]
        % atan2는 내부적으로 u_rel=0 인 상황을 완벽히 처리하므로 그대로 사용
        alpha = atan2(w_rel, u_rel);
        
        % 3. 옆미끄럼각 (beta) 계산 [rad]
        % asin() 대신 atan2()를 사용하여 부동소수점 오차에 의한 복소수 반환을 원천 차단
        den_beta = sqrt(u_rel^2 + w_rel^2);
        beta = atan2(v_rel, den_beta);

%{        
        % Lookup Table의 입력 범위를 초과하지 않도록 클램핑(Clamping)
        % Datcom 테이블이 -30도 ~ +30도까지
        alpha = max(min(alpha, 40), -40);
        beta  = max(min(beta, 40), -40);
%}

    end
    % --- 예외 처리 끝 ---
end