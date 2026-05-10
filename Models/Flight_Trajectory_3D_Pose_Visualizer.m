%% B.JET Real-time Integrated Visualizer (Right-Handed, Vector Axes)
% 워크스페이스에 'out' 객체(out.Euler_angles, out.XYZ_E)가 있다고 가정합니다.
close all;
clear v; % 🚨 [추가] 이전 실행의 찌꺼기 비디오 객체를 완벽히 제거

%% 1. 데이터 추출 및 전처리 
if isa(out.Euler_angles, 'timeseries')
    t_sim = out.Euler_angles.Time;
    euler_data = out.Euler_angles.Data;
else
    t_sim = out.tout;
    euler_data = out.Euler_angles;
end
euler_data = squeeze(euler_data);
if size(euler_data, 1) == 3 && size(euler_data, 2) > 1
    euler_data = euler_data'; 
end
euler_rad = deg2rad(euler_data); 

if isa(out.XYZ_E, 'timeseries')
    xyz_data = out.XYZ_E.Data;
else
    xyz_data = out.XYZ_E;
end
xyz_data = squeeze(xyz_data);
if size(xyz_data, 1) == 3 && size(xyz_data, 2) > 1
    xyz_data = xyz_data'; 
end

N = min(length(t_sim), size(xyz_data, 1));
t_sim = t_sim(1:N);
euler_rad = euler_rad(1:N, :);
x_E = xyz_data(1:N, 1);
y_E = xyz_data(1:N, 2);
z_E = xyz_data(1:N, 3);

%% ⏱️ 2. 실시간 동기화 파라미터 계산 
fps = 30; 
target_times = (t_sim(1) : 1/fps : t_sim(end))';
frame_indices = zeros(length(target_times), 1);
for k = 1:length(target_times)
    [~, idx] = min(abs(t_sim - target_times(k)));
    frame_indices(k) = idx;
end

%% 3. Figure 및 3D 환경 설정
target_width = 1280; target_height = 720;
fig = figure('Name', 'B.JET Flight Trajectory & 3D Pose Visualizer', 'Color', 'w');
set(fig, 'Position', [100, 100, target_width, target_height]);

hold on; grid on; box on;
daspect([1 1 1]); 
set(gca, 'ZDir', 'reverse', 'YDir', 'reverse'); 
view(45, 20);

ref_len = max([max(x_E)-min(x_E), max(y_E)-min(y_E), max(abs(z_E))]) * 0.15;
if ref_len == 0 || isnan(ref_len)
    ref_len = 10; 
end

margin = ref_len;
xlim([min(x_E)-margin, max(x_E)+margin]);
ylim([min(y_E)-margin, max(y_E)+margin]);
zlim([min(z_E)-margin, max(z_E)+margin]);

xlabel('North [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('East [m]', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Down [m]', 'FontSize', 12, 'FontWeight', 'bold');

%% 🛠️ 4. 객체 초기화 (화살표 모양)
hE_Axes = quiver3([0 0 0], [0 0 0], [0 0 0], ... 
                  [ref_len 0 0], [0 ref_len 0], [0 0 ref_len], ... 
                  0, ... 
                  'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize', 0.5);

text(ref_len*1.1, 0, 0, '$X_E$', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5]);
text(0, ref_len*1.1, 0, '$Y_E$', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5]);
text(0, 0, ref_len*1.1, '$Z_E$', 'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5]);

hTraj = plot3(NaN, NaN, NaN, 'k--', 'LineWidth', 1.5);

body_axis_len = ref_len * 0.6; 

% 🚨 [오류 방지] NaN 대신 0으로 완벽 초기화
hBX = quiver3(0, 0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.8);
hBY = quiver3(0, 0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.8);
hBZ = quiver3(0, 0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.8);

pNose = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

label_dist_ratio = 1.3;
hText_XB = text(0, 0, 0, '$X_B$', 'Interpreter', 'latex', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 12);
hText_YB = text(0, 0, 0, '$Y_B$', 'Interpreter', 'latex', 'Color', 'g', 'FontWeight', 'bold', 'FontSize', 12);
hText_ZB = text(0, 0, 0, '$Z_B$', 'Interpreter', 'latex', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 12);

%% 🎥 5. VideoWriter 설정
% 🚨 [오류 방지] 특수문자 및 공백 제거로 OS 권한 거부 완벽 차단
video_filename = 'B.JET Flight Trajectory & 3D Pose Visualizer_0424(TVC 알파,베타On).mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps; 
v.Quality = 100;
open(v);

disp(['🎥 화살표 축 버전 실시간 동기화 녹화를 시작합니다: ', video_filename]);

%% 🎬 6. 애니메이션 루프 (안전장치 적용)
tic; 

try % 🚨 [추가] 에러가 발생해도 비디오 파일을 안전하게 닫아주는 try-catch 블록
    for k = 1:length(frame_indices)
        i = frame_indices(k);
        
        cx = x_E(i); cy = y_E(i); cz = z_E(i);
        set(hTraj, 'XData', x_E(1:i), 'YData', y_E(1:i), 'ZData', z_E(1:i));
        
        C_b2e = eul2rotm(euler_rad(i, :), 'ZYX');
        
        X_vec = C_b2e * [1;0;0] * body_axis_len;
        Y_vec = C_b2e * [0;1;0] * body_axis_len;
        Z_vec = C_b2e * [0;0;1] * body_axis_len;
        
        set(hBX, 'XData', cx, 'YData', cy, 'ZData', cz, 'UData', X_vec(1), 'VData', X_vec(2), 'WData', X_vec(3));
        set(hBY, 'XData', cx, 'YData', cy, 'ZData', cz, 'UData', Y_vec(1), 'VData', Y_vec(2), 'WData', Y_vec(3));
        set(hBZ, 'XData', cx, 'YData', cy, 'ZData', cz, 'UData', Z_vec(1), 'VData', Z_vec(2), 'WData', Z_vec(3));
        
        set(pNose, 'XData', cx + X_vec(1), 'YData', cy + X_vec(2), 'ZData', cz + X_vec(3));
        set(hText_XB, 'Position', [cx + X_vec(1)*label_dist_ratio, cy + X_vec(2)*label_dist_ratio, cz + X_vec(3)*label_dist_ratio]);
        set(hText_YB, 'Position', [cx + Y_vec(1)*label_dist_ratio, cy + Y_vec(2)*label_dist_ratio, cz + Y_vec(3)*label_dist_ratio]);
        set(hText_ZB, 'Position', [cx + Z_vec(1)*label_dist_ratio, cy + Z_vec(2)*label_dist_ratio, cz + Z_vec(3)*label_dist_ratio]);
        
        title(sprintf('B.JET Flight Trajectory & 3D Pose Visualizer | Time: %.2f s\nPosition: [%.1f, %.1f, %.1f] m | Pose: [%.1f, %.1f, %.1f] deg', ...
              target_times(k), cx, cy, -cz, euler_data(i,1), euler_data(i,2), euler_data(i,3)));
        
        drawnow;
        
        frame = getframe(fig);
        img = imresize(frame.cdata, [target_height, target_width]);
        writeVideo(v, img);
        
        expected_time = target_times(k) - target_times(1);
        elapsed_time = toc;
        if elapsed_time < expected_time
            pause(expected_time - elapsed_time);
        end
    end
    
    close(v);
    disp('✅ 화살표 축 및 정보가 적용된 고품질 애니메이션 저장이 완료되었습니다!');

catch ME
    % 🚨 [핵심] 루프 도중 어떤 에러가 발생해도 이 구문이 실행되어 파일을 무조건 놓아줍니다.
    close(v); 
    disp('❌ 애니메이션 렌더링 중 오류가 발생하여 파일을 안전하게 닫았습니다.');
    rethrow(ME); % 어떤 에러인지 명령창에 표시
end