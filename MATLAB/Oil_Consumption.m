function [Oil_cons] = Oil_Consumption(Act_b, Act_b_old, Oil_cons_old)
    movement = abs(Act_b(1)-Act_b_old(1)) + abs(Act_b(2)-Act_b(2));
    Oil_cons = Oil_cons_old + movement * 11/(600 * pi/180);
    
