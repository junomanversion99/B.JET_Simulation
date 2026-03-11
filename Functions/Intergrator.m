function [v_B, w_B, C_e2b, XYZ_E, z_E] = Intergrator(v_dot_B, w_dot_B, C_dot_e2b, t)

persistent vB_state wB_state C_state XYZ_state t_prev is_init   % 상태 업데이트를 위해 state 변수 추가: 현재까지 적분된 변수들 저장

%% 함수 처음 실행 시 초기화
if isempty(is_init)                 % is_init는 함수 처음 실행인 지 확인 isempty(비어있는가?)가 참일 시 if문 실행
    vB_state  = zeros(3,1);         % zeros(3,1): [0;0;0]
    wB_state  = zeros(3,1);
    C_state   = eye(3);             % eye(3): 3x3의 단위 행렬
    XYZ_state = zeros(3,1);

    t_prev  = t;                    % t_prev는 현재 시간 저장
    is_init = true;
end

%% dt 계산
dt = t - t_prev;                    % 현재시간 - 직전 호출 시간
if dt < 0                           % 리셋, 재시작, 비정상 호출 보호장치
    dt = 0;
end
t_prev = t;

%% 입력 형상 정리 -> 열벡터로 정리
v_dot_B = v_dot_B(:);
w_dot_B = w_dot_B(:);

%% v, w 적분 
vB_state = vB_state + dt * v_dot_B;     % 상태변수에 새로운 상태 업데이트
wB_state = wB_state + dt * w_dot_B;

%% DCM 적분 + 정규직교화
C_state = C_state + dt * C_dot_e2b;
C_state = orthonormalize_dcm(C_state);  % 시간이 지나며 자세가 이상해지는걸 방지하기 위해 정규직교화

%% 위치 적분 (Earth 좌표계)
%   v_B = C_e2b * v_E  =>  v_E = C_e2b' * v_B
v_E = (C_state.') * vB_state;           % .': 전치 행렬
XYZ_state = XYZ_state + dt * v_E;

%% 출력
v_B   = vB_state;
w_B   = wB_state;
C_e2b = C_state;
XYZ_E = XYZ_state;

% (7) 고도 z_E 추출
z_E = XYZ_E(3);     % 고도 = z 성분 (모델 정의에 따라 부호가 다르면 여기만 수정)

end

%% DCM 정규직교화 함수 생성
function C_ortho = orthonormalize_dcm(C)
c1 = C(:,1);                            % C의 1번째 열을 c1으로 뽑음
c2 = C(:,2);                            % C의 2번째 열을 c2으로 뽑음

c1 = c1 / max(norm(c1), 1e-12);         % max(a, b)는 둘 중 더 큰 것을 반환 / 회전 행렬의 축벡터는 길이가 정확히 1 / 따라서 크기로 나눠줌

c2 = c2 - c1 * (c1.' * c2);             % c2를 c1에 직교하게 만든다
c2 = c2 / max(norm(c2), 1e-12);         % 회전 행렬의 축벡터는 길이가 정확히 1 / 따라서 크기로 나눠줌

c3 = cross(c1, c2);                     % c3는 외적으로 만들어 자동으로 c1, c2에 직교하도록
c3 = c3 / max(norm(c3), 1e-12);         % 회전 행렬의 축벡터는 길이가 정확히 1 / 따라서 크기로 나눠줌

C_ortho = [c1 c2 c3];                   % 정리된 축벡터 3개를 열로 붙여서 정상화된 회전행렬로 돌려줌

% 컴퓨터로 적분할 시 회전행렬에서 조금씩 벗어나므로 회저행렬로 다시 맞춰주는 과정
end