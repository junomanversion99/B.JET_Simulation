function [Q,V_mag] = Calculate_Q(rho, V_rel)
    % 이 함수는 공기 밀도(rho)와 상대 속도 벡터(V_rel)를 받아 동압(Q)을 계산합니다.
    % V_rel: [u; v; w] 형태의 3x1 벡터라고 가정합니다.
    
    % 1. 속도 벡터의 크기(Magnitude) 계산
    % V_rel이 스칼라(TAS)라면 단순히 V_mag = V_rel; 로 작성해도 됩니다.
    V_mag = norm(V_rel); 
    
    % 2. 동압 공식 적용: Q = 0.5 * rho * V^2
    Q = 0.5 * rho * V_mag^2;
end