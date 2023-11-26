%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

L = 0.7;
h = 0.3;
l = 0.1;

%% Define MDH Table

syms q1 q2 q3 q4 q1d q2d q3d q4d q1dd q2dd q3dd q4dd g real

TableMDHsym = define_table(q1, q2, q3, q4);

%% Define Station and Base Reference Frames

R_B = eye(3);
P_B = [0 -l h]';
T_B2S = buildT(R_B, P_B);
T_S2B = inv_trans(T_B2S);

R_T = eye(3);

P_T = [0.05 0 0]';

T_T2W = buildT(R_T, P_T);

%% Compute Jacobian Matrix

T_W2Bsym = simplify(dirkine(TableMDHsym));
T_T2Ssym = where_fun(T_S2B, T_W2Bsym, T_T2W);
Xsym = simplify(trans2pose(T_T2Ssym));

Jsym = simplify(jacobian(Xsym, [q1 q2 q3 q4]));

%% Define links and joints properties

Upper_Arm=link(0.46, 40, 2, 3.5);
Fore_Arm=link(0.44, 40, 2, 3.5);

Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4));
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4));
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4));
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4));



% %% Testing for Inverse Kinematics
% 
% X0 = double(subs(Xsym, [q1, q2, q3, q4], [pi/12 pi/9, -pi/4, pi/4]));
% 
% tic
% Q = invkine(X0, [q1, q2, q3, q4], Xsym);
% stopwatch = toc;
% 
% X1 = double(subs(Xsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
% 
% fprintf('\nThe desired pose was:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X0(:))
% fprintf('\nThe retrieved joint variables yield this new pose:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X1(:))
% fprintf('\nThe time required was: %.2f s\n', stopwatch)


%% Equations on Dynamics (Load side)

% Rotation matrix beetween reference frame
R_B21=R3(q1);
R_122=R3(q2)*[1 0 0; 0 0 1; 0 -1 0];
R_223=R3(q3);
R_32W=R3(q4);
R_W2T=eye(3);
R(:,:,1)=R_B21;
R(:,:,2)=R_122;
R(:,:,3)=R_223;
R(:,:,4)=R_32W;
R(:,:,5)=R_W2T;

% Path vectors (vectors which link each joint with the next one)
P_B21=[0 0 0]'; % in the sdr B
P_122=[0 0 0]'; % in the sdr 1
P_223=[Upper_Arm.Length 0 0]'; % in the sdr 2
P_32W=[Fore_Arm.Length 0 0]'; % in the sdr 3
P_W2T=[P_T(1) 0 0]'; % in the sdr 4 (wrist frame)
P(:,1)=P_B21;
P(:,2)=P_122;
P(:,3)=P_223;
P(:,4)=P_32W;
P(:,5)=P_W2T;

% CDM vectors (vectors which link each joint with the CDM of the next link attached to the joint)
P_12c1=[0 0 0]'; % in the sdr 1
P_22c2=[0 0 0]'; % in the sdr 2
P_32c3=[Upper_Arm.Length/2 0 0]'; % in the sdr 3
P_42c4=[Fore_Arm.Length/2 0 0]'; % in the sdr 4 (wrist frame)
Pc(:,1)=P_12c1;
Pc(:,2)=P_22c2;
Pc(:,3)=P_32c3;
Pc(:,4)=P_42c4;

% Mass proprerty of the link
m=[0 0 0 Upper_Arm.Mass Fore_Arm.Mass]'; % link mass (at CDM)
I=zeros(3,3,5);
I(:,:,4)=Upper_Arm.Inertia;
I(:,:,5)=Fore_Arm.Inertia;

% Derivatives of Joint variables
qd=[0 q1d q2d q3d q4d];
qdd=[0 q1dd q2dd q3dd q4dd];

% Outward iterations: 0 (Base) -> 1 -> 2 -> 3 -> 4 (Wrist)

% Preallocation
omega=sym(zeros(3, 5));
omegad=sym(zeros(3, 5));
vd=sym(zeros(3, 5)); 
vd(:,1)=sym([0 0 g]'); 
vcd=sym(zeros(3, 5));
vcd(:,1)=sym([0 0 g]'); 
F=sym(zeros(3, 5));
N=sym(zeros(3, 5));
for i=1:4
    [omega(:,i+1), omegad(:,i+1), vd(:,i+1), vcd(:,i+1)] = velAcc(R(:,:,i), omega(:,i), omegad(:,i), qd(i+1),...
                                                                 qdd(i+1), vd(:,i), [0 0 1]', P(:,i), Pc(:,i));
    [F(:,i+1), N(:,i+1)] = externalForcesTorques(m(i+1), vcd(:,i+1), omega(:,i+1), omegad(:,i+1), I(:,:,i+1));
end

% Inward iterations: T (Tool) ->  4 (Wrist) -> 3 -> 2 -> 1

% Preallocation 
f=sym(zeros(3, 5));
n=sym(zeros(3, 5));
tau=sym(zeros(5, 1));

for i=5:-1:2
    [f(:,i-1), n(:,i-1), tau(i-1)] = jointForcesTorques(R(:,:,i)', f(:,i), n(:,i), F(:,i-1), N(:,i-1), P(:,i), Pc(:,i-1),...
                                                         [0 0 1]');
end 
tau(5)=[];

TAU=vpa(simplify(tau), 3);

M=sym(ones(4,4)); % Preallocation
for i=1:4
    for j=2:5
    M(i,j)=vpa(collect(TAU(i,1), qdd(j)),3);
%     G(i,1)=collect(TAU(i,1), g);
    end
end

% %% Plots
% 
% close all
% 
% % Set the Joint Variables
% Q = [pi/12 pi/9, -pi/4, pi/4];
% 
% % Compute Necessary Variables
% 
% TableMDH = double(subs(TableMDHsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
% T_W2B = double(subs(T_W2Bsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
% T_T2S = double(subs(T_T2Ssym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
% X = double(subs(Xsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
% 
% % Create the Workspace
% figure('name', 'Workspace Test', 'WindowState', 'maximized')
% show_frame(zeros(6, 1), 'k', "S")                   % Station frame
% show_frame([P_B; zeros(3, 1)], '#2b31ed', "B")      % Base frame
% show_frame(X, '#e84f1c', "T")                       % Tool frame
% plot3DBox(L, 0.5, h);                               % Create the Box
% show_plane([1 0 0], [0 1 0], [0 0 0])               % Create the Ground
% 
% T_12B = tableRow2T(TableMDH(1, :));
% T_12S = T_B2S * T_12B;
% T_221 = tableRow2T(TableMDH(2, :));
% T_22S = T_B2S * T_12B * T_221;
% T_322 = tableRow2T(TableMDH(3, :));
% T_32S = T_B2S * T_12B * T_221 * T_322;
% T_W23 = tableRow2T(TableMDH(4, :));
% T_W2S = T_B2S * T_12B * T_221 * T_322 * T_W23;
% T_T2W = [eye(3), P_T; 0 0 0 1];
% % T_T2S = T_B2S * T_12B * T_221 * T_322 * T_W23 * T_T2W;
% show_frame(trans2pose(T_12S), "g", "1")
% show_frame(trans2pose(T_22S), "c", "2")
% show_frame(trans2pose(T_32S), "m", "3")
% show_frame(trans2pose(T_W2S), "#EDB120", "W")
% DrawJoint(0.03, 0.05, 0.1, T_12S) % first joint
% DrawJoint(0.03, 0.05, 0.1, T_22S) % second joint
% DrawJoint(0.03, 0.05, 0.1, T_32S) % third joint
% DrawJoint(0.03, 0.05, 0.1, T_W2S) % fourth joint
% DrawLink(Upper_Arm, T_22S); % first link
% DrawLink(Fore_Arm, T_32S);  % second link
% plotScoop(P_T(1), T_12S, T_22S, T_W2S) % Scoop
% legend('Station frame', 'Base frame', 'Tool frame', '$1^{st}$ joint frame',...
%     '$2^{nd}$ joint frame', '$3^{rd}$ joint frame', '$4^{th}$ joint frame (wrist frame)',...
%     'fontsize', 12,'Interpreter', 'latex')
% 
