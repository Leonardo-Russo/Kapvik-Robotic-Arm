function R = R2(angle, unit)

if nargin < 2
    unit = "rad";
end

if unit == "deg"
    angle = deg2rad(angle);
end

if unit ~= "deg" && unit ~= "rad"
    error('Wrong Unit provided as input.')
end

% Compute Rotation Matrix
R = [cos(angle)     0       -sin(angle);
     0              1       0;
     sin(angle)     0       cos(angle)];


end