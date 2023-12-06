function frames = show_jointframes(T_12S, T_22S, T_32S)
% Description: this function plots the joints reference frames in the
% enviroment.

J1 = show_frame(T_12S, "g", "1");             % 1st Joint Frame
J2 = show_frame(T_22S, "c", "2");             % 2nd Joint Frame
J3 = show_frame(T_32S, "m", "3");             % 3rd Joint Frame

% Return Frames
frames = struct('name', "Joint Frames");
frames.J1 = J1;
frames.J2 = J2;
frames.J3 = J3;

end