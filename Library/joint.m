classdef Joint
    % Description: This class includes joints properties
    properties
        Mass {mustBePositive}                       % [kg]
        Gear_Ratio {mustBePositive}                 % [adim]
        Theta_min {mustBeNegative}                  % [deg]
        Theta_max {mustBePositive}                  % [deg]
        Friction_Torque_max {mustBePositive}        % [Nm]
        Friction_Torque_min {mustBeNegative}        % [Nm]
    end

    methods
        function obj = Joint(Mass, Gear_Ratio, Theta_min, Theta_max, Friction_Torque_max, Friction_Torque_min)
            obj.Mass = Mass;
            obj.Gear_Ratio = Gear_Ratio;                             % [kg]
            obj.Theta_min = Theta_min;                               % [adim]
            obj.Theta_max = Theta_max;                               % [deg]
            obj.Theta_max = Theta_max;                               % [deg]
            obj.Friction_Torque_max = Friction_Torque_max;           % [Nm]
            obj.Friction_Torque_min = Friction_Torque_min;           % [Nm]
        end
    end
end