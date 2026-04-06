%% B.JET Rocket 6-DOF Pose Visualizer & Video Exporter (Axes-only version)
% 워크스페이스에 'out' 객체가 있다고 가정합니다.

%% 1. 데이터 추출 및 전처리
if isa(out.Euler_angles, 'timeseries')
    time = out.Euler_angles.Time;
    euler_data = out.Euler_angles.Data;
else
    time = out.tout;
    euler_data = out.Euler_angles;
end

if size(euler_data, 1) == 3 && size(euler_data, 2) > 3
    euler_data = squeeze(euler_data)'; 
elseif size(euler_data, 1) == 3 && size(euler_data, 3) > 1
    euler_data = squeeze(euler_data)';
end

euler_rad = deg2rad(euler_data); 

%% 2. 3D Figure 및 비디오 설정
% 해상도 오류 방지를 위한 핵심 설정
target_width = 1280;
target_height = 720;
fig_pos = [100, 100, target_width, target_height];

fig = figure('Name', 'B.JET Rocket 3D Pose', 'Color', 'w');
set(fig, 'Position', fig_pos, 'Units', 'pixels'); % Figure 크기 고정

hold on; grid on; axis equal;
axis_length = 1.5; % 축 선의 길이
label_offset = 1.7; % 라벨(텍스트)이 표시될 거리 (축 끝보다 약간 멀리)
xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);

% NED 좌표계 직관성을 위한 축 반전 설정
set(gca, 'ZDir', 'reverse'); 
set(gca, 'YDir', 'reverse'); 
view(45, 30); 

% 기본 축 이름
xlabel('North', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('East', 'FontSize', 12, 'FontWeight', 'bold'); 
zlabel('Down', 'FontSize', 12, 'FontWeight', 'bold');

%% 3. 정적 좌표계 (Earth NED Frame) 및 라벨 표시 ($X_E, Y_E, Z_E$)
% 회색 대시선으로 지면 좌표계 고정축 그리기
plot3([0 axis_length], [0 0], [0 0], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.5);
plot3([0 0], [0 axis_length], [0 0], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.5);
plot3([0 0], [0 0], [0 axis_length], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.5);

% 지면 좌표계 라벨 고정 (LaTeX 인터프리터 사용)
text(label_offset, 0, 0, '$X_E$', 'Interpreter', 'latex', 'FontSize', 16, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5]);
text(0, label_offset, 0, '$Y_E$', 'Interpreter', 'latex', 'FontSize', 16, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5]);
text(0, 0, label_offset, '$Z_E$', 'Interpreter', 'latex', 'FontSize', 16, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5]);

%% 4. 애니메이션 객체 (Body Frame) 초기화
% X축(빨강-Roll), Y축(초록-Pitch), Z축(파랑-Yaw)을 두꺼운 선으로 표시
pX = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 4); 
pY = plot3([0 0], [0 0], [0 0], 'g', 'LineWidth', 4);
pZ = plot3([0 0], [0 0], [0 0], 'b', 'LineWidth', 4);

% 기수(Nose) 방향 마커
pNose = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% 동적 기체 좌표계 라벨 ($X_B, Y_B, Z_B$) 초기화
hText_XB = text(0, 0, 0, '$X_B$', 'Interpreter', 'latex', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'r');
hText_YB = text(0, 0, 0, '$Y_B$', 'Interpreter', 'latex', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'g');
hText_ZB = text(0, 0, 0, '$Z_B$', 'Interpreter', 'latex', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'b');

%% 5. VideoWriter 설정 및 저장 시작
video_filename = 'B_JET_Rocket_3D_Pose.mp4';
v = VideoWriter(video_filename, 'MPEG-4'); 
v.FrameRate = 30; % 초당 30프레임
v.Quality = 100;  % 화질 100%
open(v);

disp(['🎥 3D 로켓 비디오 저장을 시작합니다: ', video_filename]);

%% 6. 애니메이션 루프 및 프레임 캡처
% 동영상 녹화를 위한 프레임 스킵 설정 (전체 데이터 중 300프레임만 캡처 = 10초 영상)
skip_step = max(1, round(length(time) / 300)); 

for i = 1:skip_step:length(time)
    % 현재 스텝의 회전 행렬(Body to Earth) 계산
    C_b2e = eul2rotm(euler_rad(i, :), 'ZYX');
    
    % 기체 좌표계의 기본 축 방향
    e1 = [1; 0; 0]; e2 = [0; 1; 0]; e3 = [0; 0; 1];
    
    % 지면 좌표계로 회전 변환된 기체 축의 끝점 계산
    X_vec = C_b2e * e1 * axis_length;
    Y_vec = C_b2e * e2 * axis_length;
    Z_vec = C_b2e * e3 * axis_length;
    
    % 1. 바디 축(선) 업데이트
    set(pX, 'XData', [0 X_vec(1)], 'YData', [0 X_vec(2)], 'ZData', [0 X_vec(3)]);
    set(pY, 'XData', [0 Y_vec(1)], 'YData', [0 Y_vec(2)], 'ZData', [0 Y_vec(3)]);
    set(pZ, 'XData', [0 Z_vec(1)], 'YData', [0 Z_vec(2)], 'ZData', [0 Z_vec(3)]);
    
    % 2. 노즈 마커 업데이트
    set(pNose, 'XData', X_vec(1), 'YData', X_vec(2), 'ZData', X_vec(3));
    
    % 3. 바디 라벨 위치 업데이트 (축 끝점보다 살짝 더 먼 곳에 텍스트 배치)
    Label_X_vec = C_b2e * e1 * label_offset;
    Label_Y_vec = C_b2e * e2 * label_offset;
    Label_Z_vec = C_b2e * e3 * label_offset;
    
    set(hText_XB, 'Position', Label_X_vec);
    set(hText_YB, 'Position', Label_Y_vec);
    set(hText_ZB, 'Position', Label_Z_vec);
    
    % 제목 업데이트
    title(sprintf('B.JET Flight Simulation | Time: %.2f sec\nYaw: %.1f, Pitch: %.1f, Roll: %.1f [deg]', ...
          time(i), euler_data(i,1), euler_data(i,2), euler_data(i,3)));
    
    drawnow; % 화면 실시간 렌더링
    
    % 📸 4. 프레임 캡처 및 강제 리사이징 (VideoWriter 에러 방지)
    frame = getframe(fig); 
    img = imresize(frame.cdata, [target_height, target_width]); 
    writeVideo(v, img);
end

%% 7. 비디오 저장 종료
close(v); 
disp('✅ 로켓 3D 애니메이션 동영상이 저장되었습니다! 현재 폴더를 확인해 주세요.');