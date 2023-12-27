function G = Gravity(q_1, q_2, q_3, q_4)
g=3.71;

G =g*[                                                                                                                                                                                                0;
cos(q_2 + q_3 + q_4)/20 + (5143078768285974084362884352152049*cos(q_2 + q_3))/6338253001141147007483516026880000 + (669967009185953507743449959841840399*cos(q_2))/405648192073033408478945025720320000;
                                                                                       cos(q_2 + q_3 + q_4)/20 + (5143078768285974084362884352152049*cos(q_2 + q_3))/6338253001141147007483516026880000;
                                                                                                                                                                                cos(q_2 + q_3 + q_4)/20];
  
end