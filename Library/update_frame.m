function update_frame(frame, Xnew)

% Retrieve New Pose from Input
r_new = Xnew(1:3);
roll_new = Xnew(4);
pitch_new = Xnew(5);
yaw_new = Xnew(6);

% Calculate New Frame Orientation
alpha = 0.15;   % axis length
F_new = R1(roll_new) * R2(pitch_new) * R3(yaw_new) * alpha * eye(3);

% Update Frame Origin
set(frame.O, 'XData', r_new(1), 'YData', r_new(2), 'ZData', r_new(3));

% Update Arrows
set(frame.arw1, 'XData', r_new(1), 'YData', r_new(2), 'ZData', r_new(3), 'UData', F_new(1, 1), 'VData', F_new(1, 2), 'WData', F_new(1, 3));
set(frame.arw2, 'XData', r_new(1), 'YData', r_new(2), 'ZData', r_new(3), 'UData', F_new(2, 1), 'VData', F_new(2, 2), 'WData', F_new(2, 3));
set(frame.arw3, 'XData', r_new(1), 'YData', r_new(2), 'ZData', r_new(3), 'UData', F_new(3, 1), 'VData', F_new(3, 2), 'WData', F_new(3, 3));

% Update Text Positions
set(frame.txt1, 'Position', [r_new(1) + F_new(1, 1), r_new(2) + F_new(1, 2), r_new(3) + F_new(1, 3)]);
set(frame.txt2, 'Position', [r_new(1) + F_new(2, 1), r_new(2) + F_new(2, 2), r_new(3) + F_new(2, 3)]);
set(frame.txt3, 'Position', [r_new(1) + F_new(3, 1), r_new(2) + F_new(3, 2), r_new(3) + F_new(3, 3)]);

% Update the Plot
drawnow;

end

   

