function [start, goal, speed, velocities] = sim_setup(r,x0,y0,n)
global scenario_info_speed; global scenario_info_velocities; global scenario_info_start; global scenario_info_goal; global v_max;
for m = 1:n
    H_a(1,m) = randi([-180,180])*(pi/180); % heading angles
%         if n == 15
%             speed(1,m) = 4*sqrt(2);
%         elseif n == 25
%             speed(1,m) = 6*sqrt(2);
%         else
%             speed(1,m) = 7*sqrt(2);
%         end
    speed(1,m) = sqrt(2)*v_max;
    velocities(m,1) = speed(1,m)*cos(H_a(1,m));
    velocities(m,2) = speed(1,m)*sin(H_a(1,m));
end

t = linspace(-pi,pi,n+1);
xi = r*cos(t) + x0;
yi = r*sin(t) + y0;
xj = (r+r/2)*cos(t) + x0;
yj = (r+r/2)*sin(t) + y0;
points = [];
points2 = [];
for k = 1:length(xi)
points = [points;xi(k),yi(k)];
points2 = [points2;xj(k),yj(k)];
end

points = unique(points,'rows');

if length(points) ~= n
    points(1,:)=[]; %since -pi and pi should the have same x,y values. 
    points2(1,:)=[];
end

for k =1:size(points,1)
    start(k,:) = [randi([0,1])*50,randi([0,1])*50] + points2(k,:);
end

temp = rot90(points);
goal = rot90(temp);

scenario_info_speed = speed;
scenario_info_velocities = velocities;
scenario_info_start = start;
scenario_info_goal = goal;
