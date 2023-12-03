%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

options = struct('name', "Options");
options.show_frames = true;


%% Define links and joints properties

% Box Parameters
L = 0.7;                % m
w = 0.5;                % m
h = 0.3;                % m

% Base Shift
P_By = 0.1;             % m

% Scoop Dimension
scoopLength = 0.1;      % m

% Other Parameters
a2 = 0.46;              % m
a3 = 0.44;              % m

% Create Links
Upper_Arm = link(0.46, 40, 2, 3.5);
Fore_Arm = link(0.44, 40, 2, 3.5);

% Create Joints
Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);

% Define d3
d3 = 1.5*10^(-3)*Upper_Arm.Diameter;


%% Define MDH Table

syms q1 q2 q3 q4 q1d q2d q3d q4d q1dd q2dd q3dd q4dd g real

TableMDHsym = define_table(q1, q2, q3, q4, a2, a3, d3);

%% Define Station and Base Reference Frames

R_B = eye(3);
P_B = [0 -P_By h]';
T_B2S = buildT(R_B, P_B);
T_S2B = inv_trans(T_B2S);

R_T = eye(3);

P_T = [scoopLength 0 0]';

T_T2W = buildT(R_T, P_T);

%% Compute Jacobian Matrix

T_W2Bsym = simplify(dirkine(TableMDHsym));
T_T2Ssym = where_fun(T_S2B, T_W2Bsym, T_T2W);
X_Tsym = simplify(trans2pose(T_T2Ssym));

Jsym = simplify(jacobian(X_Tsym, [q1 q2 q3 q4]));

% %% Test ikineAnal
% XS2T=double(subs(X_Tsym, [q1, q2, q3, q4], [pi/6 -pi/12, +pi/4, pi/12])); % vector from station to the tool in the station sdr
% xB2T=double(subs(T_W2Bsym(1:3,4), [q1, q2, q3, q4], [pi/6 -pi/12, +pi/4, pi/12])); % vector from base to the tool in the station sdr
% X=[xB2T; XS2T(4:6)];
% [Q] = invkinAnal(X, Upper_Arm.Length, Fore_Arm.Length, "ElbowUp");
% (Q(1))
% (Q(2))
% (Q(3))
% (Q(4))


% %% Testing for Inverse Kinematics numeric
% 
% % X0 = double(subs(Xsym, [q1, q2, q3, q4], [pi/12 pi/9, -pi/4, pi/4]));
% % X0 = [0.8 ,	0 ,	0.8 , 1.5708 ,	0.1543 ,	0.1988]
% X0 = [0.8 ,	0 ,	0.8]
% %[0.1988 	0.8692 	-0.4873 	-0.5363]
% %[0.1988 	0.2941 	0.6306 	-0.7704]
% 
% tic
% Qinv = invkine(X0, [q1, q2, q3, q4], X_Tsym);
% stopwatch = toc;
% 
% X1 = double(subs(X_Tsym, [q1, q2, q3, q4], [Qinv(1), Qinv(2), Qinv(3), Qinv(4)]));
% 
% fprintf('\nThe desired pose was:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X0(:))
% fprintf('\nThe joint variables are:\n [%.4f \t%.4f \t%.4f \t%.4f]\n', Qinv(:))
% fprintf('\nThe retrieved joint variables yield this new pose:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X1(:))
% fprintf('\nThe time required was: %.2f s\n', stopwatch)

%% Dynamical equations (symbolic expression)
[M, V, G, F] = dinEqs(Joint_1, Joint_2, Joint_3, Joint_4, Upper_Arm, Fore_Arm, P_T);

% %% Trajectory Generation (Stowage to Navigation)
% NSto2Nav=10; % number of via points
% TSto2Nav=10; % total time from stowage to Navigation [s]
% ft=1000; % path update rate [Hz]
% thetaddMax=deg2rad(40); % [rad/s^2] me la sono inventata, va aggiustata
% 
% q0Sto2Nav=[0 0 0 0]'; % It is necessary to start from joint space because of the intial pose singularity (q3 offset of 180°)
% % Pose at via points
% % We have to decide X for all via points from Stowage to Navigation
% for i=1:N-1
%     [QSto2Nav(:,i)] = invkinAnal(X(:,i), L1, L2, "SuEuc2p"); % shoulder up + elbow up + cos(q2) positivo
% end
% [tSto2Nav, qSto2Nav, qdSto2Nav, qddSto2Nav] = trajectoryGeneration(q0Sto2Nav, QSto2Nav, thetaddMax, TSto2Nav, NSto2Nav, ft);
% 
% % Plot 
% figure(1)
% plot(t,rad2deg(qSto2Nav(1,:)),'r',lineWidth=1.5)
% hold on
% plot(t,rad2deg(qSto2Nav(2,:)),'b',lineWidth=1.5)
% plot(t,rad2deg(qSto2Nav(3,:)),'g',lineWidth=1.5)
% plot(t,rad2deg(qSto2Nav(4,:)),'m',lineWidth=1.5)
% xlabel('$$t$$ [s]','Interpreter','latex')
% ylabel('$$\theta(t)$$ [^\circ]','Interpreter','latex')
% legend('Joint 1 ($$\theta_1$$)', 'Joint 2 ($$\theta_2$$)', 'Joint 3 ($$\theta_3$$)',...
%     'Joint 4 ($$\theta_45$$)', 'Interpreter','latex')
% title('Joint angles','Interpreter','latex')
% set(gca,'FontSize',20)
% 
% figure(2)
% plot(t,rad2deg(qdSto2Nav(1,:)),'r',lineWidth=1.5)
% hold on
% plot(t,rad2deg(qdSto2Nav(2,:)),'b',lineWidth=1.5)
% plot(t,rad2deg(qdSto2Nav(3,:)),'g',lineWidth=1.5)
% plot(t,rad2deg(qdSto2Nav(4,:)),'m',lineWidth=1.5)
% xlabel('$$t$$ [s]','Interpreter','latex')
% ylabel('$$\dot\theta(t)$$ [^\circ/s]','Interpreter','latex')
% legend('Joint 1 ($$\dot\theta_1$$)', 'Joint 2 ($$\dot\theta_2$$)', 'Joint 3 ($$\dot\theta_3$$)',...
%     'Joint 4 ($$\dot\theta_4$$)', 'Interpreter','latex')
% title('Joint velocities','Interpreter','latex')
% set(gca,'FontSize',20)
% 
% figure(3)
% plot(t,rad2deg(qddSto2Nav(1,:)),'r',lineWidth=1.5)
% hold on
% plot(t,rad2deg(qddSto2Nav(2,:)),'b',lineWidth=1.5)
% plot(t,rad2deg(qddSto2Nav(3,:)),'g',lineWidth=1.5)
% plot(t,rad2deg(qddSto2Nav(4,:)),'m',lineWidth=1.5)
% xlabel('$$t$$ [s]','Interpreter','latex')
% ylabel('$$\ddot\theta(t)$$ [^\circ/s^2]','Interpreter','latex')
% legend('Joint 1 ($$\ddot\theta_1$$)', 'Joint 2 ($$\ddot\theta_2$$)', 'Joint 3 ($$\ddot\theta_3$$)',...
%     'Joint 4 ($$\ddot\theta_4$$)', 'Interpreter','latex')
% title('Joint accelerations','Interpreter','latex')
% set(gca,'FontSize',20)

%% Plot Initial Condition

close all

% Set the Initial Joint Variables
Q = [pi/12 pi/9, -pi/4, pi/4];

% Compute Necessary Variables
TableMDH = double(subs(TableMDHsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
T_W2B = double(subs(T_W2Bsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
T_T2S = double(subs(T_T2Ssym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
X_T = double(subs(X_Tsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

T_12B = tableRow2T(TableMDH(1, :));
T_12S = T_B2S * T_12B;
T_221 = tableRow2T(TableMDH(2, :));
T_22S = T_B2S * T_12B * T_221;
T_322 = tableRow2T(TableMDH(3, :));
T_32S = T_B2S * T_12B * T_221 * T_322;
T_W23 = tableRow2T(TableMDH(4, :));
T_W2S = T_B2S * T_12B * T_221 * T_322 * T_W23;
T_T2W = [eye(3), P_T; 0 0 0 1];
% T_T2S = T_B2S * T_12B * T_221 * T_322 * T_W23 * T_T2W;

X_S = zeros(6, 1);
X_B = trans2pose(T_B2S);
X_W = trans2pose(T_W2S);

X_1 = trans2pose(T_12S);
X_2 = trans2pose(T_22S);
X_3 = trans2pose(T_32S);

% Create the Workspace
figure('name', 'Enviroment Simulation')

% Add toggle button for Legend
toggleButton = uicontrol('Style', 'pushbutton', 'String', 'Show Legend', 'Position', [20 20 100 20]);

env = show_env(L, w, h);
joints = show_joints(T_12S, T_22S, T_32S, T_W2S);
links = show_links(Upper_Arm, T_22S, Fore_Arm, T_32S);
scoop = plotScoop(scoopLength, T_12S, T_22S, T_W2S);

if options.show_frames
    mframes = show_mainframes(X_S, X_B, X_W, X_T);
    jframes = show_jointframes(X_1, X_2, X_3);
end

axis equal

% Choose Material: [ambient, diffuse, specular, shininess, specularExponent]
material([0.5, 0.6, 0.6, 0, 0.2]);

% Set Light
light('Position', [2 2 5], 'Style', 'local');
lightangle(150, 40);
lighting gouraud

if options.show_frames
    lgd = legend([mframes.S.arw1, mframes.B.arw1, mframes.W.arw1, mframes.T.arw1, jframes.J1.arw1, jframes.J2.arw1, jframes.J3.arw1], ...
                 {'Station Frame', 'Base Frame', 'Wrist Frame', 'Tool Frame', ...
                  '1^{st} Joint Frame', '2^{nd} Joint Frame', '3^{rd} Joint Frame'}, ...
                  'location', 'best', 'fontsize', 8);
    
    set(lgd, 'Visible', 'off');     % by default legend is hidden

    % Set the callback for the button
    set(toggleButton, 'Callback', {@toggleLegendCallback, lgd});

end


%% Plot Live Evolution

% Set the Joint Variables

N = 100;
q1_span = linspace(pi/12, pi/3, N)';
q2_span = linspace(pi/9, pi/3, N)';
q3_span = linspace(-pi/4, -pi/3, N)';

input('Press Enter to Start the Simulation...\n');

for i = 1 : N

    Q = [q1_span(i) q2_span(i), q3_span(i), pi/4];

    % Compute Necessary Variables
    TableMDH = double(subs(TableMDHsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
    T_W2B = double(subs(T_W2Bsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
    T_T2S = double(subs(T_T2Ssym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
    X_T = double(subs(X_Tsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

    T_12B = tableRow2T(TableMDH(1, :));
    T_12S = T_B2S * T_12B;
    T_221 = tableRow2T(TableMDH(2, :));
    T_22S = T_B2S * T_12B * T_221;
    T_322 = tableRow2T(TableMDH(3, :));
    T_32S = T_B2S * T_12B * T_221 * T_322;
    T_W23 = tableRow2T(TableMDH(4, :));
    T_W2S = T_B2S * T_12B * T_221 * T_322 * T_W23;
    T_T2W = [eye(3), P_T; 0 0 0 1];
    % T_T2S = T_B2S * T_12B * T_221 * T_322 * T_W23 * T_T2W;

    X_S = zeros(6, 1);
    X_B = trans2pose(T_B2S);
    X_W = trans2pose(T_W2S);

    X_1 = trans2pose(T_12S);
    X_2 = trans2pose(T_22S);
    X_3 = trans2pose(T_32S);


    % Update the Plot
    update_joint(joints.J1, T_12S);
    update_joint(joints.J2, T_22S);
    update_joint(joints.J3, T_32S);
    update_joint(joints.J4, T_W2S);

    update_link(links.arm, T_22S, Upper_Arm);
    update_link(links.forearm, T_32S, Fore_Arm);

    update_scoop(scoop, scoopLength, T_W2S, T_12S, T_22S);

    if options.show_frames
        update_frame(mframes.S, X_S);
        update_frame(mframes.B, X_B);
        update_frame(mframes.W, X_W);
        update_frame(mframes.T, X_T);

        update_frame(jframes.J1, X_1);
        update_frame(jframes.J2, X_2);
        update_frame(jframes.J3, X_3);
    end

    pause(1/N)

end
