function links = show_links(Arm, T_22S, ForeArm, T_32S)
% Description: this function plots the links in the enviroment.

arm = DrawLink(Arm, T_22S);           % Arm
forearm = DrawLink(ForeArm, T_32S);        % ForeArm

% Return Joints Structure
links = struct('name', "Links");
links.arm = arm;
links.forearm = forearm;

end

