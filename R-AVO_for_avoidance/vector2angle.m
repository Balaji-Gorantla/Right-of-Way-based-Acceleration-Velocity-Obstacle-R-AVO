function angle_in_degrees = vector2angle(u,v)
  a= sqrt(u(1)^2+u(2)^2);
  b=sqrt(v(1)^2+v(2)^2);
  angle_in_degrees=acos(dot(u,v)/(a*b))*180/pi;
end