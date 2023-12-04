function update_scoop(S, L, T_12S, T_T2S)

% Retrieve the Pose
X = trans2pose(T_T2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
roll = rad2deg(X(4));
pitch = rad2deg(X(5));
yaw = rad2deg(X(6));

% Define the vertices of the box
x = r(1);
y = r(2);
z = r(3);

edges = [x-L, y+L/2, z+L;
         x+L, y+L, z+L;
         x+L, y+L, z-L;
         x-L, y+L/2, z-L;
         x-L, y-L*0.8, z+L;
         x+L, y-L*0.8, z+L;
         x+L, y-L*0.8, z-L;
         x-L, y-L*0.8, z-L];

% Update the vertices of the box
set(S, 'Vertices', [edges(:, 1), edges(:, 2), edges(:, 3)]);

% Apply rotations
rotate(S, [0 0 1], yaw, T_T2S(1:3,4));
rotate(S, T_12S(1:3,2), pitch, T_T2S(1:3,4));
rotate(S, T_T2S(1:3,1), roll, T_T2S(1:3,4));

end
