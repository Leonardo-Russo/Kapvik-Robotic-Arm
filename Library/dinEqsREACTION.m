function [M, V, G, tauF] = dinEqsREACTION(Joint_1, Joint_2, Joint_3, Joint_4, UpperArm, ForeArm, P_T, Mscoop, Iscoop, d3)
% Dynamic eqs 

format long
syms q_1 q_2 q_3 q_4 q_1d q_2d q_3d q_4d q_1dd q_2dd q_3dd q_4dd g Tc1 Tc2 Tc3 Tc4 f5x f5y f5z n5x n5y n5z real
sympref('AbbreviateOutput', false);
JOINT=[Joint_1 Joint_2 Joint_3 Joint_4];
% Mass property of the link
ml=[0 0 UpperArm.Mass ForeArm.Mass Mscoop]'; % link mass 
mj=[0 Joint_1.Mass Joint_2.Mass Joint_3.Mass Joint_4.Mass]'; % joint mass
m=ml+mj; % total mass
I=zeros(3,3,5);
I(:,:,3)=UpperArm.Inertia;
I(:,:,4)=ForeArm.Inertia;
I(:,:,5)=Iscoop;

% Rotation matrix beetween reference frame
R_B21=R3(q_1);
R_122=R3(q_2)*[1 0 0; 0 0 1; 0 -1 0];
R_223=R3(q_3);
R_32W=R3(q_4);
R_W2T=eye(3);
R(:,:,1)=R_B21;
R(:,:,2)=R_122;
R(:,:,3)=R_223;
R(:,:,4)=R_32W;
R(:,:,5)=R_W2T;

% Path vectors (vectors which link each joint with the next one)
P_B21=[0 0 0]'; % in the sdr B
P_122=[0 0 0]'; % in the sdr 1
P_223=[UpperArm.Length 0 d3]'; % in the sdr 2
P_32W=[ForeArm.Length 0 0]'; % in the sdr 3
P_W2Tend=[P_T(1); -P_T(1)/2; 0]; % in the sdr 4 (wrist frame)
P(:,1)=P_B21;
P(:,2)=P_122;
P(:,3)=P_223;
P(:,4)=P_32W;
P(:,5)=P_W2Tend;

% CDM vectors (vectors which link each joint with the CDM of the next link attached to the joint)
P_02c0=[0 0 0]'; % from base to cdm of link beetween base and joint 1, expressed in the sdr of the base
P_12c1=[0 0 0]'; % from joint 1 to cdm of link beetween joint 1 and joint 2, expressed in the sdr 1
P_22c2=[(UpperArm.Length/2)*(ml(3)/m(3)) 0 0]'; % from joint 2 to cdm of link beetween joint 2 and joint 3, expressed in the sdr 2
P_32c3=[(ForeArm.Length/2)*(ml(4)/m(4)) 0 0]'; % from joint 3 to cdm of link beetween joint 3 and joint 4, expressed in the sdr 3
P_42c4=[P_T(1)*(ml(5)/m(5)) 0 0]'; % from joint 4 to cdm of the escavator, expressed in the sdr 4 (wrist frame)
Pc(:,1)=P_02c0;
Pc(:,2)=P_12c1;
Pc(:,3)=P_22c2;
Pc(:,4)=P_32c3;
Pc(:,5)=P_42c4;

% Derivatives of Joint variables
qd=[0 q_1d q_2d q_3d q_4d];
qdd=[0 q_1dd q_2dd q_3dd q_4dd];

% Outward iterations: 0 (Base) -> 1 -> 2 -> 3 -> 4 (Wrist)

% Preallocation
omega=sym(zeros(3, 5));
omegad=sym(zeros(3, 5));
vd=sym(zeros(3, 5)); 
vd(:,1)=sym([0 0 g]'); 
vcd=sym([0 0 g]'); 
F=sym(zeros(3, 5));
N=sym(zeros(3, 5));
for i=1:4
    [omega(:,i+1), omegad(:,i+1), vd(:,i+1), vcd(:,i+1)] = velAcc(R(:,:,i), omega(:,i), omegad(:,i), qd(i+1),...
                                                                 qdd(i+1), vd(:,i), [0 0 1]', P(:,i), Pc(:,i+1));
    [F(:,i+1), N(:,i+1)] = externalForcesTorques(m(i+1), vcd(:,i+1), omega(:,i+1), omegad(:,i+1), I(:,:,i+1));
end

% Inward iterations: T (Tool) ->  4 (Wrist) -> 3 -> 2 -> 1

% Preallocation 
f=sym(zeros(3, 5));
n=sym(zeros(3, 5));
f(:,5)=[f5x f5y f5z]';
n(:,5)=[n5x n5y n5z]';
tau=sym(zeros(5, 1));

for i=5:-1:2
    [f(:,i-1), n(:,i-1), tau(i-1)] = jointForcesTorques(R(:,:,i)', f(:,i), n(:,i), F(:,i), N(:,i), P(:,i), Pc(:,i),...
                                                         [0 0 1]');
end 
tau(5)=[];

TAU=collect(simplify(tau),[q_1dd q_2dd q_3dd q_4dd g]);

M=sym(zeros(4,4)); % Preallocation
G=sym(zeros(4,1)); % Preallocation

for i=1:4
    for j=1:4
        mij=coeffs(TAU(i), qdd(j+1));
        gi=coeffs(TAU(i), g);
        if isempty(mij)
            M(i,j)=0;
        elseif length(mij)==2
            M(i,j)=mij(end);
        elseif length(mij)==1
            M(i,j)=0;
        elseif length(mij) > 2
            warning('Coeffs Sfaciola :)');
        end
        if isempty(gi)
            G(i,1)=0;
        elseif length(gi)==2
            G(i,1)=gi(end);
        elseif length(gi)==1
            G(i,1)=0;
        elseif length(gi) > 2
            warning('Coeffs Sfaciola :)');
        end
    end
end

qsym=[q_1 q_2 q_3 q_4]';
C=sym(zeros(4,4,4));
c=sym(zeros(4,1));
M=simplify(M);
for i=1:4
    C(:,:,i)=(1/2)*(jacobian(M(:,i),qsym)+jacobian(M(:,i),qsym)')-diff(M,qsym(i));
    c(i)=qd(2:5)*(C(:,:,i))*qd(2:5)';
end
V=(c);

Mm=sym(zeros(4,4)); % Preallocation
for i=1:4
    Mm(i,i)=(JOINT(i).Gear_Ratio^2)*JOINT(i).Motor_Inertia;
end

tauF=sym(ones(4,1)); % Preallocation
for i=1:4
    if i==1
        Tcoul=Tc1;
    elseif i==2
        Tcoul=Tc2;
    elseif i==3
        Tcoul=Tc3;
    elseif i==4
        Tcoul=Tc4;
    end
    tauF(i,1)=-(JOINT(i).Gear_Ratio^2)*JOINT(i).B_m*qd(i+1)-Tcoul;
end

M=M+Mm;
% M=vpa(M,3);
% V=vpa(V,3);
% G=vpa(G,3);
% tauF=vpa(tauF,3);


end