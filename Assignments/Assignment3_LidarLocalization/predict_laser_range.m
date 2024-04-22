function y_exp = predict_laser_range(row, col, theta, beam_angle, map, ogres, thresh)
    
    new_angle = atan2(sin(theta+beam_angle), cos(theta+beam_angle));
    incr  = 0;
    r_p = row;
    c_p = col;
    
    if -pi/4<=new_angle && new_angle<=pi/4   
        c_inc = 1;
        r_inc = tan(new_angle);
        a = cos(new_angle);
    elseif 3*pi/4<=new_angle || new_angle<=-3*pi/4
        c_inc = -1;
        r_inc = -tan(new_angle);
        a = cos(new_angle);
    elseif pi/4<new_angle && new_angle<3*pi/4 
        c_inc = 1/tan(new_angle);
        r_inc = 1;
        a = sin(new_angle);
    else
        c_inc = -1/tan(new_angle);
        r_inc = -1;
        a = sin(new_angle);

    end


    [row_bound, col_bound] = size(map);
    while r_p > 0 && c_p>0 && r_p<=row_bound && c_p<=col_bound && map(r_p, c_p) < thresh
        incr = incr +1;
        r_p = row + round(incr * r_inc);
        c_p = col + round(incr * c_inc);
    end
    y_exp = abs((incr/a)*ogres);
end