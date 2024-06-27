function region = get_conflict_region(B1,B2,B3,B4, angle)
    region = 0;
    
    if (angle > B1) && (angle <= B2)
       region = 2;
    elseif (angle > B2) && (angle < B3)
        region = 3;
    elseif (angle >= B3) && (angle < B4)
        region = 4;
    else
        region = 1;
    end
    
end