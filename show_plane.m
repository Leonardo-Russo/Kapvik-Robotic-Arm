function show_plane(u, v, p)
% Description: this function draws the plane normal to the input vector v
% and at point P.

% THIS IS A WORK IN PROGRESS

C = 0.5;

[P,Q] = meshgrid(-C:C);     % provide a gridwork

% Compute the corresponding cartesian coordinates using the two vectors in w
x1 = p(1);
y1 = p(2);
z1 = p(3);

X = x1 + u(1)*P + v(1)*Q;
Y = y1 + u(2)*P + v(2)*Q; 
Z = z1 + u(3)*P + v(3)*Q;

surf(X,Y,Z, 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25)

% alpha 0.2

end