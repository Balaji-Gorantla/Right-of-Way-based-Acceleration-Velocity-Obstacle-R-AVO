function ang = angle_finder(v,v_des)

    cur_ang = atan2(v(2),v(1)); % current velocity angle
    if cur_ang<0
       cur_ang = cur_ang+2*pi;
    end

    des_ang = atan2(v_des(2),v_des(1)); % desired velocity angle
    if des_ang<0
       des_ang = des_ang+2*pi;
    end
    
    ang = mod((des_ang - cur_ang + pi ),2*pi) + pi; 
    if ang>2*pi
        ang = ang - 2*pi;
    end

end