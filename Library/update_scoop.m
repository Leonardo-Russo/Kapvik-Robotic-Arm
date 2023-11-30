function update_scoop(S, length, T_W2S, T_12S, T_22S)

% Retrieve the Pose
X = trans2pose(T_W2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
roll = rad2deg(X(4));
pitch = rad2deg(X(5));
yaw = rad2deg(X(6));

% Define the vertices of the box
x = [r(1)-length, r(1)+length, r(1)+length, r(1)-length, r(1)-length, r(1)+length, r(1)+length, r(1)-length];
y = [r(2), r(2), r(2)-2*length, r(2)-2*length, r(2), r(2), r(2)-2*length, r(2)-2*length];
z = [r(3)-length, r(3)-length, r(3)-length, r(3)-length, r(3)+length/3, r(3)+length/3, r(3)+length, r(3)+length];

% Update the vertices of the box
set(S, 'Vertices', [x', y', z']);

% Apply rotations
rotate (S, [0 0 1], yaw, T_W2S(1:3,4));
rotate (S, T_12S(1:3,2), pitch, T_W2S(1:3,4));
rotate (S, T_22S(1:3,2), roll, T_W2S(1:3,4));

end
