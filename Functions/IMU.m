function [a_E, w_E]= IMU(F_total_B, mass_current, C_e2b, w_B)
%#codegen


%% 입력 차원(Shape) 강제 고정
% Simulink가 간혹 1D Array로 신호를 넘길 때 발생하는 차원 에러를 원천 차단!
F_total_B = reshape(F_total_B, [3, 1]);
w_B       = reshape(w_B,       [3, 1]);
C_e2b     = reshape(C_e2b,     [3, 3]);


% Acceleration 
a_inertial_B = F_total_B / mass_current ;
a_E_col = ( C_e2b.') * a_inertial_B ; % Acceleraiton 포트로 연결

a_E  = a_E_col.'; % 전치시켜줘서 IMU model 입력 배열 형식 맞춤

% Angular Velocity
w_E_col = ( C_e2b.') * w_B ; % 각속도 포트로 연결

w_E = w_E_col.'; % 전치시켜줘서 IMU model 입력 배열 형식 맞춤