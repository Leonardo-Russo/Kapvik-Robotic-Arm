function joints = show_joints(T_12S, T_22S, T_32S, T_W2S)
% Description: this function plots the joints reference frames in the
% enviroment.

J1 = DrawJoint(0.05, 0.07, 0.18, T_12S);        % 1st Joint
J2 = DrawJoint(0.05, 0.07, 0.18, T_22S);        % 2nd Joint
J3 = DrawJoint(0.05, 0.07, 0.18, T_32S);        % 3rd Joint
J4 = DrawJoint(0.05, 0.07, 0.18, T_W2S);        % 4th Joint

% Return Joints
joints = struct('name', "Joints");
joints.J1 = J1;
joints.J2 = J2;
joints.J3 = J3;
joints.J4 = J4;

end

