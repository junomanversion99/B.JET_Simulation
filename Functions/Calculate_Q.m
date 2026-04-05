function [Q,V_mag] = Calculate_Q(rho, V_rel)
    % 이 함수는 공기 밀도(rho)와 상대 속도 벡터(V_rel)를 받아 동압(Q)을 계산함
    %% 입력
    % rho   : 고도Z_E에 따른 공기밀도  [kg/m^3] 스칼라
    % V_rel : 바디좌표계에 투영한 로켓의 대기에 대한 상대속도 [m/s],  3x1

    %% 출력
    % Q     : 동압 [Pa = kg/(m*s^2)] 스칼라
    % V_mag : V_rel 의 크기(norm) [m/s] 스칼라
    
    % 1. 속도 벡터의 크기(Magnitude) 계산
    V_mag = norm(V_rel); 
    
    % 2. 동압 공식 적용: Q = 0.5 * rho * V^2
    Q = 0.5 * rho * V_mag^2;
end