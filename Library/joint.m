classdef joint
    % Description: This class includes joints properties
    properties
        Mass {mustBePositive}                       % [kg]
        Gear_Ratio {mustBePositive}                 % [adim]
        Theta_min {mustBeNegative}                  % [deg]
        Theta_max {mustBePositive}                  % [deg]
        Friction_Torque_max {mustBePositive}        % [Nm]
        Friction_Torque_min {mustBeNegative}        % [Nm]
        Motor_Inertia {mustBePositive}              % [kgm^2]
        Tau_m_max {mustBePositive}                  % [Nm]
        i_tau_m_max {mustBePositive}                % [A]
        B_m {mustBePositive}                        % [Nmsrad^-1]
        RPM_max {mustBePositive}                    % [RPM]
        Accuracy {mustBePositive}                   % [°]
    end 

    methods
        function obj = joint(Mass, Gear_Ratio, Theta_min, Theta_max, Friction_Torque_max, Friction_Torque_min, ...
                             Motor_Inertia, Tau_m_max, i_tau_m_max, B_m, RPM_max, Accuracy)
            obj.Mass = Mass;                                         % [kg]
            obj.Gear_Ratio = Gear_Ratio;                             % [adim]
            obj.Theta_min = Theta_min;                               % [deg]
            obj.Theta_max = Theta_max;                               % [deg]
            obj.Friction_Torque_max = Friction_Torque_max;           % [Nm]
            obj.Friction_Torque_min = Friction_Torque_min;           % [Nm]
            obj.Motor_Inertia=Motor_Inertia;                         % [kgm^2]
            obj.Tau_m_max=Tau_m_max*10^(-3);                         % [Nm]
            obj.i_tau_m_max=i_tau_m_max;                             % [A]
            obj.B_m=B_m;                                             % [Nmsrad^-1]
            obj.RPM_max=RPM_max;                                     % [RPM]
            obj.Accuracy=Accuracy;                                   % [°]
        end
    end
end