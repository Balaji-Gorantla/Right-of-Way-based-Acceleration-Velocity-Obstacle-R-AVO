function is_right = right_left(P,obj,v_pref)

% A = [(obj(1) - P(1)), (v_pref(1)- P(1));
%     (obj(2) - P(2)), (v_pref(2)-P(2))];

A = [(v_pref(1)- obj(1)), (v_pref(2)- obj(2));
    (P(1)- obj(1)), (P(2)-obj(2))];

result = det(A);

if result > 0
    is_right = -1;
elseif result < 0
   is_right =  1;
else
   is_right = -1;
end

end