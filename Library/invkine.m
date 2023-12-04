function [Q] = invkine(X, L1, L2, d, typeOfSol)
% This function take as input the pose X of the wrist wrt the base in the 
% base reference frame and the link length and gives as output the
% corresponding joint angles that correspond to the particular solution
% specificated in "typeOfSol"

%% Pose OF THE WRIST, NOT OF THE TOOL
x=X(1);
y=X(2);
z=X(3);
roll=X(4);
pitch=X(5); % [rad]
yaw=X(6); % [rad]
prec=yaw; % [rad]
[q234] = RPY2angle(roll, pitch, yaw); % sequence 3-1-3

%% q1
q1=prec;

%% q3
c3=(- L1^2 - L2^2 + x^2 + y^2 + z^2 + d^2)/(2*L1*L2);
s3(1)=sqrt(1 - c3^2); % elbow up
s3(2)=-sqrt(1 - c3^2); % elbow down
q3(1)=atan2(s3(1),c3); % elbow up
q3(2)=atan2(s3(2),c3); % elbow down

%% q2
if abs(prec)<10^(-4) || abs(prec-pi)<10^(-4)|| abs(prec+pi)<10^(-4) % sin(yaw) circa 0, uso l'elemento della matrice 1,4 piuttosto che 2,4
    c2(1)=(L1*x + L2*c3*x - L1*d*sin(prec) - L2*c3*d*sin(prec) + L2*s3(1)*z*cos(prec))/...
        (cos(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(1)^2)); % elbow up
    s2(1)=(L1*z*cos(prec) - L2*s3(1)*x + L2*c3*z*cos(prec) + L2*d*s3(1)*sin(prec))/...
        (cos(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(1)^2)); % elbow up
    c2(2)=(L1*x + L2*c3*x - L1*d*sin(prec) - L2*c3*d*sin(prec) + L2*s3(2)*z*cos(prec))/...
        (cos(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(2)^2)); % elbow down
    s2(2)=(L1*z*cos(prec) - L2*s3(2)*x + L2*c3*z*cos(prec) + L2*d*s3(2)*sin(prec))/...
        (cos(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(2)^2)); % elbow down
else
    c2(1)=(L1*y + L2*c3*y + L1*d*cos(prec) + L2*c3*d*cos(prec) + L2*s3(1)*z*sin(prec))/...
        (sin(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(1)^2)); % elbow up
    s2(1)=(L1*z*sin(prec) - L2*s3(1)*y - L2*d*s3(1)*cos(prec) + L2*c3*z*sin(prec))/...
        (sin(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(1)^2)); % elbow up
    c2(2)=(L1*y + L2*c3*y + L1*d*cos(prec) + L2*c3*d*cos(prec) + L2*s3(2)*z*sin(prec))/...
        (sin(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(2)^2)); % elbow down
    s2(2)=(L1*z*sin(prec) - L2*s3(2)*y - L2*d*s3(2)*cos(prec) + L2*c3*z*sin(prec))/...
        (sin(prec)*(L1^2 + 2*L1*L2*c3 + L2^2*c3^2 + L2^2*s3(2)^2)); % elbow down
end

q2(1)=atan2(s2(1),c2(1)); % elbow up
q2(2)=atan2(s2(2),c2(2)); % elbow down

%% q4
q4(1)=q234-q2(1)-q3(1);
q4(2)=q234-q2(2)-q3(2);

%% Solution
if typeOfSol=="ElbowUp" 
    Q=[q1 q2(1) q3(1) q4(1)]';
elseif typeOfSol=="ElbowDown"
    Q=[q1 q2(2) q3(2) q4(2)]';
end

if ~isreal(Q)
    error('Joint angles must be real!!')
end
    
end