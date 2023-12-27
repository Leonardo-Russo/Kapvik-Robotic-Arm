function [zT] = ToolHeight(q1, q2, q3, q4)

zT=0.05*cos(q2 + q3 + q4 - 1.0471975511965977461542144610932) + 0.44*sin(q2 + q3) + 0.46*cos(q2) + 0.39;

end