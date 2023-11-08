function TableMDH = defineTable(q1, q2, q3, q4)
% Description: this is a utility function which defines the MDH Table
% outside of the main enviroment

a2 = 0.46;      % m
a3 = 0.44;      % m

TableMDH = [0,      0,     0,     q1;...
            pi/2,   0,     0,     q2;...
            0,      a2,    0,     q3;...
            0,      a3,    0,     q4];

showTable(TableMDH)

end