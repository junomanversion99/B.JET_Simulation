function [F_thrust, MOI_current, CG_current, mass_current] = Propulsion(t, Thrust_raw, Mass, Prop)
    %% 입력:
    % t          : 시뮬레이션 시간 [s] (Clock 블록) 
    % Thrust_raw : Lookup Table에서 온 추력 값 [N]
    % Mass       : 질량/관성 Structer (Mass.mass_initial 등)
    % Prop       : 추진 Structer (Prop.mass_dot_negative 등)

    %% 출력:
    % F_thrust      : 기체좌표계에 투영시킨 추력 [N] 3x1
    % MOI_current   : 현재 관성모멘트
    % CG_current    : 현재 CG 위치
    % mass_current  : 현재 질량



    t_burn = Prop.burn_time;    % 연소종료시간

    if t <= t_burn
        % 1. 연소 중: 1차 함수 모델링 (상수 기울기 적용)
        mass_current = Mass.mass_initial - Prop.mass_dot_negative * t;
        MOI_current  = Mass.MOI_initial - Prop.MOI_dot_negative * t;
        CG_current   = Prop.CG_initial - Prop.CG_dot_negative * t;
        F_thrust     = [Thrust_raw;0;0];
    else
        % 2. 연소 종료 후: 최종 값으로 고정
        mass_current = Mass.mass_final;
        MOI_current  = Mass.MOI_final;
        CG_current   = Prop.CG_final;
        F_thrust     = [0;0;0];
    end
end