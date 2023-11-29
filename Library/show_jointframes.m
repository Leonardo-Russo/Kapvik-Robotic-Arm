function frames = show_jointframes(X_1, X_2, X_3)
% Description: this function plots the joints reference frames in the
% enviroment.

J1 = show_frame(X_1, "g", "1");             % 1st Joint Frame
J2 = show_frame(X_2, "c", "2");             % 2nd Joint Frame
J3 = show_frame(X_3, "m", "3");             % 3rd Joint Frame

% Return Frames
frames = struct('name', "Joint Frames");
frames.J1 = J1;
frames.J2 = J2;
frames.J3 = J3;

end