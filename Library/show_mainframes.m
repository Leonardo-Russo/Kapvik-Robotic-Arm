function frames = show_mainframes(T_S2S, T_B2S, T_W2S, T_T2S)
% Description: this function plots the main reference frames in the
% enviroment.

S = show_frame(T_S2S, 'k', "S");              % Station frame
B = show_frame(T_B2S, '#2b31ed', "B");        % Base frame
W = show_frame(T_W2S, "#EDB120", "W");        % Wrist Frame
T = show_frame(T_T2S, '#e84f1c', "T");        % Tool frame

% Return Frames
frames = struct('name', "Main Frames");
frames.S = S;
frames.B = B;
frames.W = W;
frames.T = T;

end