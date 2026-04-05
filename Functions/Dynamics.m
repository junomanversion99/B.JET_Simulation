function [v_dot_B, w_dot_B, C_dot_e2b, XYZ_dot_E, Euler_angles] = Dynamics(v_B, w_B, C_e2b, F_total_B, M_total_B, mass_current, MOI_current)

    %% 출력 크기 명시
    v_dot_B   = zeros(3,1);
    w_dot_B   = zeros(3,1);
    C_dot_e2b = zeros(3,3);

    %% 입력 형상 강제
    v_B       = reshape(v_B,       [3,1]);
    w_B       = reshape(w_B,       [3,1]);
    F_total_B = reshape(F_total_B, [3,1]);
    M_total_B = reshape(M_total_B, [3,1]);
    C_e2b     = reshape(C_e2b,     [3,3]);

    %% 관성모멘트 행렬 형상 정리
    if isequal(size(MOI_current), [3 3])
        I = MOI_current;
    elseif numel(MOI_current) == 9
        I = reshape(MOI_current, [3,3]);
    else
        error('MOI_current must be 3x3 or 9 elements.');
    end

    %% 병진 운동
    v_dot_B = F_total_B / mass_current - cross(w_B, v_B);

    %% 회전 운동
    H = I * w_B;
    w_dot_B = I \ (M_total_B - cross(w_B, H));  % AGCL 검토 완.

    %% 자세행렬 미분
    wx = w_B(1);
    wy = w_B(2);
    wz = w_B(3);

    omega = [  0   -wz   wy;
              wz    0   -wx;
             -wy   wx    0 ];

    C_dot_e2b = -omega * C_e2b;

    %% Euler angle 추출
    phi   = atan2(C_e2b(2,3), C_e2b(3,3)); 
    theta = asin(-C_e2b(1,3));              
    psi   = atan2(C_e2b(1,2), C_e2b(1,1)); 

    Euler_angles = [phi; theta; psi];  % [3x1] 벡터 [rad]
    Euler_angles = [psi; theta; phi];  % 순서바꿈. 주석으로 무슨 순서의 오일러각 사용할지 선택하면 됨.
    Euler_angles = rad2deg(Euler_angles);    % [deg]로 덮어쓰기
    
    %% 자세 미분
    XYZ_dot_E = C_e2b.' * v_B;

end

%{
function [v_dot_B, w_dot_B, C_dot_e2b] = Dynamics(v_B, w_B, C_e2b, F_total_B, M_total_B, mass_current, MOI_current)

%% 입력
% v_B        [3x1] 바디 좌표계 속도
% w_B        [3x1] 바디 좌표계 각속도 (p q r)
% C_e2b      [3x3] DCM_earth_to_body
% F_total_B  [3x1] 바디 좌표계 합력
% M_total_B  [3x1] 바디 좌표계 합모멘트
% mass_current  [1x1] 현재 질량
% MOI_current  [3x3] 현재 관성모멘트

%% 출력
% v_dot_B    [3x1] 바디 좌표계 선가속도
% w_dot_B    [3x1] 바디 좌표계 각가속도
% C_dot_e2b  [3x3] DCM_earth_to_body 미분

%% 형상 정리(행/열 혼선 방지) -> 열벡터로 정리
v_B       = v_B(:);
w_B       = w_B(:);
F_total_B = F_total_B(:);
M_total_B = M_total_B(:);

%% 병진 운동 방정식 
% SumF = m*(v_dot_B + w x v)  ->  v_dot = (1/m)*SumF - (w x v)
v_dot_B = F_total_B./mass_current - cross(w_B, v_B);

%% 관성 입력 처리 -> 혹시 몰라서
if numel(MOI_current) == 3            % [Ixx; Iyy; Izz]와 같이 주관성모멘트만 들어왔을 경우
    I = diag(MOI_current(:));         % 대각 관성행렬로 바꿈
else                            
    I = MOI_current;                  % 3×3 관성행렬일 경우 그대로 출력
end

%% 회전 운동 방정식 
% SumM = I*w_dot_B + w x (I*w)  ->  w_dot = I^(-1)*(SumM - w x (I*w))
H = I * w_B;                                  % 각운동량
w_dot_B = I \ (M_total_B - cross(w_B, H));    % 역행렬 대신 \ 사용 (좌측 행렬 나눗셈)

%% DCM 
% C_dot_e2b = omega * C_e2b -> 원동민 선배님 ppt 29p
wx = w_B(1); 
wy = w_B(2); 
wz = w_B(3);
omega = [  0   -wz   wy;
          wz    0   -wx;
         -wy   wx    0 ];    

C_dot_e2b = omega * C_e2b;          % 각속도 벡터를 행렬로 변환해서 행렬곱 실행

end
%}