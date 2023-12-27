function [React] = TauREACTION(q1, q2, q3, q4, f5, n5)

f5x=f5(1);
f5y=f5(2);
f5z=f5(3);
n5x=n5(1);
n5y=n5(2);
n5z=n5(3);

React=[- (3 * f5x * cos(q2 + q3 + q4)) / 50 - (f5z * cos(q2 + q3 + q4)) / 20 + n5y * cos(q2 + q3 + q4) ...
       + (3 * f5y * sin(q2 + q3 + q4)) / 50 - (f5z * sin(q2 + q3 + q4)) / 40 + n5x * sin(q2 + q3 + q4);

       f5x/40 + f5y/20 + n5z + (23*f5y*cos(q3+q4))/50 + (23*f5x*sin(q3+q4))/50 + ...
       (11*f5y*cos(q4))/25 + (11*f5x*sin(q4))/25;

       f5x/40 + f5y/20 + n5z + (11*f5y*cos(q4))/25 + (11*f5x*sin(q4))/25;

       f5x/40 + f5y/20 + n5z];


end