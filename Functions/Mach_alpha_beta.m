function [Mach, alpha, beta] = Mach_alpha_beta(V_rel, V_sound)
    % 입력: V_rel [u_rel; v_rel; w_rel], V_sound (음속)
    
    u_rel = V_rel(1);
    v_rel = V_rel(2);
    w_rel = V_rel(3);
    
    V_mag = sqrt(u_rel^2 + v_rel^2 + w_rel^2);
    
    % --- 예외 처리 시작 ---
    eps_v = 1e-6; % 속도 임계값 (m/s)
    
    if V_mag < eps_v
        % 속도가 거의 0인 경우 (정지 상태 등)
        Mach = 0;
        alpha = 0;
        beta = 0;
    else
        % 일반적인 비행 상태
        Mach = V_mag / V_sound;
        
        % rad2deg 함수를 사용하여 라디안을 디그리로 변환
        alpha_rad = atan2(w, u);
        alpha = rad2deg(alpha_rad); 
        
        beta_rad = asin(max(min(v / V_mag, 1), -1));
        beta = rad2deg(beta_rad);
    end
    % --- 예외 처리 끝 ---
end