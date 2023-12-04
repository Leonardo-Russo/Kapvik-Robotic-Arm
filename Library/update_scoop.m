function update_scoop(S, L, T_T2S)

% Retrieve the Pose
X = trans2pose(T_T2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
roll = X(4);
pitch = X(5);
yaw = X(6);

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

% Apply rotation matrices
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];

% Rotate vertices
for i = 1:size(edges,1)
    rotated = Rz * Ry * Rx * (edges(i,:)' - r);
    edges(i,:) = r' + rotated';
end


% Update the vertices of the box
set(S, 'Vertices', [edges(:, 1), edges(:, 2), edges(:, 3)]);


end
