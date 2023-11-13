function invT = inv_trans(T)
% Description: this function builds the inverse transformation matrix from
% a known one.

invT = zeros(4, 4);

R = T(1:3, 1:3);
D = T(1:3, 4);

invT(1:3, 1:3) = R';
invT(1:3, 4) = -R'*D;
invT(end, end) = 1;

end