function V_rel  = Calculate_V_rel( z_E, v_B, C_e2b)

%% 입력
% z_E   : 고도 [m] 
% v_B   : 바디 속도 [m/s], 3x1
% C_e2b : DCM (Earth -> Body), 3x3

%% 출력
% V_rel : 바디좌표계에서의 상대속도 [m/s], 벡터값 3x1

%% 1) 무풍 모델 (일단은 무풍 추후에 u, v 데이터값 추가)
u = 0;  v = 0;  w = 0;
wind_E = [u; v; w];          % 지면 좌표계 풍속(3x1)
wind_B = C_e2b * wind_E;     % 바디 좌표계 풍속(3x1)

%% 바람 모델 -> 추후 u, v 추가

% u = interp1(alt, wind_u, z_E, 'linear', 'extrap');
% v = interp1(alt, wind_v, z_E, 'linear', 'extrap');
% w = 0; 
% vq = interp1(x, v, xq)
% x: 이미 알고 있는 데이터의 위치 (x축 값)
% v: 이미 알고 있는 데이터의 실제 값 (y축 값)
% xq: 내가 궁금한, 값을 알고 싶은 새로운 위치 
% vq: 함수가 계산해서 알려주는 추측된 값 
% u, v는 추후 베이스 파라미터에 엑셀표로 삽입

%% 출력
V_rel = v_B - wind_B;        % 상대 속도 벡터 (Body) 3x1
end