function joints = show_joints(T_12S, T_22S, T_32S, T_W2S)
% Description: this function plots the joints in the enviroment.

Di = 0.05;      % inner diameter
Do = 0.07;      % outer diameter
L = 0.18;       % length

J1 = DrawJoint(Di, Do, L, T_12S);           % 1st Joint
J2 = DrawJoint(Di, Do, L, T_22S);           % 2nd Joint
J3 = DrawJoint(Di, Do, L, T_32S);           % 3rd Joint
J4 = DrawJoint(Di, Do, L, T_W2S);           % 4th Joint

% Return Joints Structure
joints = struct('name', "Joints");
joints.J1 = J1;
joints.J2 = J2;
joints.J3 = J3;
joints.J4 = J4;
joints.Di = Di;
joints.Do = Do;
joints.L = L;

end

