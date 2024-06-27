% function AVO_cone = avo_cone(tag,agent,others,delta,plan_cycle,R_d)
% X_disks = [];
% Y_disks = [];
% pax =  agent.Position(1);
% pay =  agent.Position(2);
% pbx = others(tag).Position(1);
% pby = others(tag).Position(2);
% ra = agent.Radius+R_d;
% rb = others(tag).Radius+R_d;
% Vax = agent.NewVelocity(1) + agent.Position(1);
% Vay = agent.NewVelocity(2) + agent.Position(2);
% vbx = others(tag).NewVelocity(1) + pbx;
% vby = others(tag).NewVelocity(2) + pby;

delta = 0.001:0.2:5;
pax = 1;
pay = 1;
pbx = 5;
pby = 1;
Vax = 5;
Vay = 5;
vbx = 4;
vby = 4;
ra = 0.5;
rb = 0.5;
t = 1;

for m = 1:length(delta)
    disk_center_x(m) = ((delta(m)*(exp(-t/delta(m))-1)*(Vax - vbx))-(pax- pbx))/(t+delta(m)*(exp(-t/delta(m))-1));
    disk_center_y(m) = ((delta(m)*(exp(-t/delta(m))-1)*(Vay - vby))-(pay- pby))/(t+delta(m)*(exp(-t/delta(m))-1));
    disk_radius(m) = (ra+rb)/((t+delta(m)*(exp(-t/delta(m))-1)));
    disk_center(m) = norm([disk_center_x(m),disk_center_y(m)]);
    1;
%     ang = 0:2*pi/1000:2*pi;
%     x_disks = disk_center_x(m) + disk_radius(m)*cos(ang);
%     y_disks = disk_center_y(m) + disk_radius(m)*sin(ang);
%     X_disks = [X_disks x_disks];
%     Y_disks = [Y_disks y_disks];
end

figure(1)
plot(delta, disk_center, Marker="+");
xlabel('tau');
ylabel('a');
figure(2)
plot(delta, disk_radius, Marker="*");
xlabel('tau');
ylabel('b');
