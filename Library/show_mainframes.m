function frames = show_mainframes(X_S, X_B, X_W, X_T)
% Description: this function plots the main reference frames in the
% enviroment.

S = show_frame(X_S, 'k', "S");              % Station frame
B = show_frame(X_B, '#2b31ed', "B");        % Base frame
W = show_frame(X_W, "#EDB120", "W");        % Wrist Frame
T = show_frame(X_T, '#e84f1c', "T");        % Tool frame

% Return Frames
frames = struct('name', "Main Frames");
frames.S = S;
frames.B = B;
frames.W = W;
frames.T = T;

end