function updatePlot(src, ~, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, Upper_Arm, Fore_Arm, scoopLength, tTrajectory, qTrajectory, ft)

global Q Qsym
% 
% q1 = Qsym(1);
% q2 = Qsym(2);
% q3 = Qsym(3);
% q4 = Qsym(4);
% 
% jointIndex = str2double(src.Tag(2));        % extract joint index from the tag
% newAngle = deg2rad(get(src, 'Value'));      % get the new value from the slider
% Q(jointIndex) = newAngle;

% Simulation loop
for i = 1:length(tTrajectory)
    % Update joint angles from the trajectory
    Q = qTrajectory(:, i);% update the corresponding joint variable

    % Compute new Manipulator State
    [T_12S, T_22S, T_32S, T_W2S, T_T2S, X_T] = ...
        getManipulatorState(Q, TableMDHsym, X_Tsym, T_B2S, T_T2W);

    T_S2S = eye(4);

    % Update the Plot
    update_joint(joints.J1, T_12S);
    update_joint(joints.J2, T_22S);
    update_joint(joints.J3, T_32S);
    update_joint(joints.J4, T_W2S);

    update_link(links.arm, T_22S, Upper_Arm);
    update_link(links.forearm, T_32S, Fore_Arm);

    update_scoop(scoop, scoopLength, T_T2S);

    if options.show_frames
        update_frame(mframes.S, T_S2S);
        update_frame(mframes.B, T_B2S);
        update_frame(mframes.W, T_W2S);
        update_frame(mframes.T, T_T2S);

        update_frame(jframes.J1, T_12S);
        update_frame(jframes.J2, T_22S);
        update_frame(jframes.J3, T_32S);
    end

    % Command Window Logs
    showQ
    fprintf('X_T\t=\t[%.3f \t%.3f \t%.3f \t%.3f \t%.3f \t%.3f]\n', X_T)
    % show_pseudodetJ     % show pseudo-determinant of Jacobian Matrix

    % Add a pause to control the speed of the simulation
    pause(1/ft);
    
end
end
