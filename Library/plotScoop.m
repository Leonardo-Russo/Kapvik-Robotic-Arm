function S = plotScoop(length, T_12S, T_22S, T_W2S, facecolor)
% Description: this function draws a box in 3D space. The reference frame
% is coherent with the choice of station frame in this case study.

if nargin < 5
    facecolor = [219, 219, 219];
    facecolor = facecolor / norm(facecolor);
end

% Retrieve the Pose
X = trans2pose(T_W2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
roll = rad2deg(X(4));
pitch = rad2deg(X(5));
yaw = rad2deg(X(6));

edgecolor = [82, 82, 82];
edgecolor = edgecolor / norm(edgecolor);

% Define the vertices of the box
% The origin is on the bottom, at the center of one of the short sides
x = [r(1)-length, r(1)+length, r(1)+length, r(1)-length, r(1)-length, r(1)+length, r(1)+length, r(1)-length];
y = [r(2), r(2), r(2)-2*length, r(2)-2*length, r(2), r(2), r(2)-2*length, r(2)-2*length];
z = [r(3)-length, r(3)-length, r(3)-length, r(3)-length, r(3)+length/3, r(3)+length/3, r(3)+length, r(3)+length];
% Define the vertices of each face
faces = [1, 2, 6, 5;    % Front face
         2, 3, 7, 6;    % Right face
         3, 4, 8, 7;    % Back face
         4, 1, 5, 8;    % Left face
         1, 2, 3, 4];   % Bottom face

% R = R1(roll,"deg")*R2(pitch,"deg")*R3(yaw,"deg");
% for i=1:8
%     newR=R*[x(i) y(i) z(i)]';
%     rx(i)=newR(1);
%     ry(i)=newR(2);
%     rz(i)=newR(3);
% end

% Plot the Box
S=patch('Vertices',[rx',ry',rz'], 'Faces',faces, 'FaceColor', facecolor, 'EdgeColor', edgecolor, 'FaceAlpha', 0.55, 'HandleVisibility','off');
rotate (S, [0 0 1], yaw, T_W2S(1:3,4));
rotate (S, T_12S(1:3,2), pitch, T_W2S(1:3,4));
rotate (S, T_22S(1:3,2), roll, T_W2S(1:3,4));


end
