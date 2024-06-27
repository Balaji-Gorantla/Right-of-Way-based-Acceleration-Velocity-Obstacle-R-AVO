clear all; close all;
V = [];
v= [0;1];
alpha = 2*pi/16;

R = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];

for i = 1:16
    V(:,i) = R*v;
    ang(i) = angle_finder([0;1],V(:,i));
    cos_ang(i) = cos(ang(i));
    sin_ang(i) = sin(ang(i));
    v = V(:,i);
end

v_des_1 = [1,1];
v_des_2 = [-1,1];

ang_1 = angle_finder(v,v_des_1)*180/pi;
ang_2 = angle_finder(v,v_des_2)*180/pi;
c_1 = cos(ang_1);
s_1 = sin(ang_1);
c_2 = cos(ang_2);
s_2 = sin(ang_2);
ang3 = -atan2(v_des_2(2),v_des_2(1))*180/pi + atan2(v(2),v(1))*180/pi;

t = 1:1:16;
figure(1)
plot(t,ang);
figure(2)
plot(ang, cos_ang);
figure(3)
plot(ang, sin_ang);
1;