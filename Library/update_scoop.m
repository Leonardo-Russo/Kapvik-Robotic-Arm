function update_scoop(S, L, T_T2S)

% Retrieve the Pose
X = trans2pose(T_T2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
R = T_T2S(1:3, 1:3);

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


% Rotate vertices
for i = 1:size(edges,1)
    rotated = R * (edges(i,:)' - r);
    edges(i,:) = r' + rotated';
end


% Update the vertices of the box
set(S, 'Vertices', [edges(:, 1), edges(:, 2), edges(:, 3)]);


end
