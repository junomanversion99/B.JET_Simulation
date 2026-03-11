function [F_thrust, MOI_current, CG_current, mass_current] = Propulsion(t, Thrust_raw, Mass, Prop)
    % 입력:
    % t          : 시뮬레이션 시간 [s] (Clock 블록)
    % Thrust_raw : Lookup Table에서 온 추력 값 [N]
    % Mass       : 질량/관성 구조체 (Mass.mass_initial 등)
    % Prop       : 추진 구조체 (Prop.m_dot_negative 등)

    t_burn = Prop.burn_time;

    if t <= t_burn
        % 1. 연소 중: 1차 함수 모델링 (상수 기울기 적용)
        mass_current = Mass.mass_initial - Prop.m_dot_negative * t;
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