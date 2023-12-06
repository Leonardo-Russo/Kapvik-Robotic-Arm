function updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, Upper_Arm, Fore_Arm, scoopLength)

global Q Qsym
q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

jointIndex = str2double(src.Tag(2));        % extract joint index from the tag
newAngle = deg2rad(get(src, 'Value'));      % get the new value from the slider
Q(jointIndex) = newAngle;                   % update the corresponding joint variable

% Compute new Manipulator State
[T_12S, T_22S, T_32S, T_W2S, T_T2S, X_S, X_B, X_W, X_T, X_1, X_2, X_3] = ...
    getManipulatorState(Q, TableMDHsym, X_Tsym, T_B2S, T_T2W);


% Update the Plot
update_joint(joints.J1, T_12S);
update_joint(joints.J2, T_22S);
update_joint(joints.J3, T_32S);
update_joint(joints.J4, T_W2S);

update_link(links.arm, T_22S, Upper_Arm);
update_link(links.forearm, T_32S, Fore_Arm);

update_scoop(scoop, scoopLength, T_T2S);

if options.show_frames
    update_frame(mframes.S, X_S);
    update_frame(mframes.B, X_B);
    update_frame(mframes.W, X_W);
    update_frame(mframes.T, X_T);

    update_frame(jframes.J1, X_1);
    update_frame(jframes.J2, X_2);
    update_frame(jframes.J3, X_3);
end

drawnow;    % visually update the plot

showQ

show_pseudodetJ

fprintf('q234\t=\t%.4f\n', Q(2)+Q(3)+Q(4))

fprintf('\nThe Tool pose is:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X_T)

end
