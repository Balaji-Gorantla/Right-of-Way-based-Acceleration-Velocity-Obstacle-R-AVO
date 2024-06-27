function AVO_cone_new = avo_cone_new(tag,agent,others,delta,plan_cycle,R_d)
X_disks = [];
Y_disks = [];
pax =  agent.Position(1);
pay =  agent.Position(2);
pbx = others(tag).Position(1);
pby = others(tag).Position(2);
ra = agent.Radius+R_d;
rb = others(tag).Radius+R_d;
% Vax = agent.OldVelocity(1) + agent.Position(1);
% Vay = agent.OldVelocity(2) + agent.Position(2);
% vbx = others(tag).OldVelocity(1) + pbx;
% vby = others(tag).OldVelocity(2) + pby;
Vax = agent.OldVelocity(1)- agent.Position(1);
Vay = agent.OldVelocity(2) - agent.Position(2);
vbx = others(tag).OldVelocity(1) - pbx;
vby = others(tag).OldVelocity(2) - pby;

t = plan_cycle(1:end);

for m = 1:length(t)
    disk_center_x(m) = (delta*(exp(-t(m)/delta)-1)*(Vax - vbx)-(pax- pbx))/(t(m)+delta*(exp(-t(m)/delta)-1));
    disk_center_y(m) = (delta*(exp(-t(m)/delta)-1)*(Vay - vby)-(pay- pby))/(t(m)+delta*(exp(-t(m)/delta)-1));
    disk_radius(m) = (ra+rb)/((t(m)+delta*(exp(-t(m)/delta)-1)));

    ang = 0:2*pi/1000:2*pi;
%     x_disks = disk_center_x(m) + disk_radius(m)*cos(ang) + others(tag).OldVelocity(1) + (others(tag).Acceleration(1))*t(m);% + (others(tag).NewVelocity(1) - exp(-t(m)/delta)*(others(tag).NewVelocity(1) - others(tag).OldVelocity(1)));
%     y_disks = disk_center_y(m) + disk_radius(m)*sin(ang) + others(tag).OldVelocity(2) + (others(tag).Acceleration(2))*t(m);% + (others(tag).NewVelocity(2) - exp(-t(m)/delta)*(others(tag).NewVelocity(2) - others(tag).OldVelocity(2)));
    
    x_disks = disk_center_x(m) + disk_radius(m)*cos(ang) - others(tag).OldVelocity(1) + delta*(others(tag).Acceleration(1))*(exp(t(m)/delta)-1);
    y_disks = disk_center_y(m) + disk_radius(m)*sin(ang) - others(tag).OldVelocity(2) + delta*(others(tag).Acceleration(2))*(exp(t(m)/delta)-1);
    X_disks = [X_disks x_disks];
    Y_disks = [Y_disks y_disks];
end

P = [X_disks(:), Y_disks(:)];
[k,av] = convhull(P);
% hold on;
% plot(P(k,1), P(k,2));


for i=1:length(k)
    temp = k(i);
    pos_X(i) = X_disks(temp);
    pos_Y(i) = Y_disks(temp);
end

% pos_X = pos_X + others(tag).NewVelocity(1) + (others(tag).Acceleration(1))*delta;
% pos_Y = pos_Y + others(tag).NewVelocity(2) + (others(tag).Acceleration(2))*delta;

% pos_X = pos_X + others(tag).NewVelocity(1) + (others(tag).NewVelocity(1) - others(tag).OldVelocity(1));
% pos_Y = pos_Y + others(tag).NewVelocity(2) + (others(tag).NewVelocity(2) - others(tag).OldVelocity(2));
% 
% pos_X = pos_X + pax;
% pos_Y = pos_Y + pay;
% 
% pos_X = pos_X + vbx;
% pos_Y = pos_Y + vby;

AVO_cone_new = [pos_X;pos_Y];
end