function R = R1(angle, unit)

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
R = [1          0               0;
     0          cos(angle)      sin(angle);
     0          -sin(angle)     cos(angle)];


end