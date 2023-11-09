classdef Link
    % Description: This class includes links properties and compute mass
    % and inertia matrix of its link in his principal reference frame
    %
    % PRINCIPAL REFERENCE FRAME OF LINK (coherent with link frame
    % definition in the slide)
    % - Center in cdm of link (at its center)
    % - x-axis = axis along the link (along the pipe)
    % - z-axis = axis aligned with first joint axis (the link is beetween 2
    %            joints, we consider the first joint)
    % - y-axis = axis to form right handed reference frame

    properties
        Length {mustBePositive}       % [m]
        Diameter {mustBePositive}     % [mm]
        Thickness {mustBePositive}    % [mm]
        Density {mustBePositive}      % [g/cm^3]
        Mass {mustBePositive}         % [kg]
        Inertia                       % [kg*m^2]
    end

    methods
        function obj = Link(Length, Diameter, Thickness, Density)
            obj.Length = Length;                                    % [m]
            obj.Diameter = Diameter;                                % [mm]
            obj.Thickness = Thickness;                              % [mm]
            obj.Density = Density;                                  % [g/cm^3]

            rin = ((obj.Diameter-(obj.Thickness*2))*10^(-3))/2;     % [m]
            s = obj.Thickness*10^(-3);                              % [m]
            L = obj.Length;                                         % [m]
            V = (2*pi*rin*s)*L;                                     % [m^3]
            rho = obj.Density*10^3;                                 % [kg/m^3]
            m = rho*V;                                              % [kg]

            rin = ((obj.Diameter-(obj.Thickness*2))*10^(-3))/2;     % [m]
            rout = obj.Diameter*10^(-3)/2;                          % [m]
            L = obj.Length;                                         % [m]
            rho = obj.Density*10^3;                                 % [kg/m^3]
            funx = @(r,theta,z)((r.*cos(theta)).^2+(r.*sin(theta)).^2);
            funz = @(r,theta,z)((r.*sin(theta)).^2+z.^2);
            funy = @(r,theta,z)((r.*cos(theta)).^2+z.^2);
            rinf = rin;
            rsup = rout;
            thetainf = 0;
            thetasup = 2*pi;
            zinf = -L/2;
            zsup = L/2;
            Ixx = rho*integral3(funx, rinf, rsup, thetainf, thetasup, zinf, zsup);
            Iyy = rho*integral3(funy, rinf, rsup, thetainf, thetasup, zinf, zsup);
            Izz = rho*integral3(funz, rinf, rsup, thetainf, thetasup, zinf, zsup);
            I=diag([Ixx Iyy Izz]);

            obj.Mass = m;                                           % [kg]
            obj.Inertia = I;                                        % [kg*m^2]
        end
    end
end