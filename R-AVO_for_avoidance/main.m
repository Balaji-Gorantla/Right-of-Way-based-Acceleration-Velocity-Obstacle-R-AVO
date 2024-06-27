function [t_mission,t_com,d_minimum,eta_min, eta_mean, a_rms, failure_count, acc_lo_rms, acc_la_rms] = main(n)
   clear all;close all;
machine = tic;
global time;global tol_d; global t_step; global xmax; global ymax;
global B1 B2 B3 B4 RoW_Matrix;
global delta; global v_max;
delta = 2;
v_max = 10*delta; % for v_max = a_max*delta/2, a_max = 2g

time = 2;
t_step = 0.3;
t_run = 0;

flag = 0;
global is_test_req;
is_test_req = true;% true false
isplot_req = false; % true
isSimulation_req = true; % false

if is_test_req
    n = 2;
end
R = 0.2;
% n = 14;
if ~is_test_req
%     if n == 10
%         S = 15;
%     elseif n == 20
%         S = 20;
%     elseif n == 30 
%         S = 25;
%     elseif n == 40
%         S = 30;
%     end
    S = 25;
    S_theta = 2*pi/n;
    r = S/(2*sin(S_theta/2)); %placing all the agents on a circle of a radius r
end
xmax = 50-100; %simulation plot axes limits
ymax = 50+100; 
x0 = 50;
y0 = 50;

% n = input('Number of Agents:');
% R = input('Radius of agents:');

B1 = 45*pi/180;
B2 = 135*pi/180;
B3 = 225*pi/180;
B4 = 315*pi/180;
% RoW_Matrix = [0 1 1 0 ;  
%               0 1 1 0;
%               0 1 1 0;
%               0 1 1 0];   %rows=alpha, coloumn=beta

RoW_Matrix = [0 1 1 1 ;  
              0 1 1 0;
              0 1 1 1;
              0 1 1 1];

tol_d = 1;

x_position_info = zeros(100000,n);
y_position_info = zeros(100000,n);
x_velocity_info = zeros(100000,n);
y_velocity_info = zeros(100000,n);
velocity_info = zeros(100000,n);
acceler = zeros(100000,n);
t_c1 = zeros(1,n);
d_min1 = zeros(1,n);
failure_count = 0;



%initializing all the agents
agents = Agent(0,[0,0],[0,0],0,[0,0],R,[0,0],[0,0],0,0,[0,0,0],[0,0],[0,0],n);
% Agent(id, pos, goal, s, p_s, r, n_p, n_p_s, n_d, n_a, c, o_v, n)


% getting starting and goal positions of each agents
if is_test_req
%     S = 16;
%     S_theta = 2*pi/n;
%     r = S/(2*sin(S_theta/2)); %placing all the agents on a circle of a radius r
%     [start, goal, speed, velocities] = sim_setup(r,x0,y0,n);
    [start, goal, speed, velocities] =  sim_setup1;
else
    [start, goal, speed, velocities] = sim_setup(r,x0,y0,n);
end



%assigning identity, start and goal points, speed and preffered velocity(direction towards goal) to each agent
for i = 1:length(agents)
    agents(i).Identity = i; 
    agents(i).Position = start(i,:);
    agents(i).Target = goal(i,:);
    agents(i).NewVelocity = velocities(i,:);
    agents(i).Velocity = speed(i);
end

%   setting speed and preferred speed of each agent
for i=1:length(agents)
%     agents(i).setVelocity;
    agents(i).setPrefVelocity;
%     agents(i).NewVelocity = agents(i).PrefVelocity;
end

for j = 1: n
   distance_td = [agents(j).Position; agents(j).Target];
   d_td = pdist(distance_td);
%     velocity = sqrt(agents(j).Velocity(1)^2 + agents(j).Velocity(2)^2);
   t_d(j) = d_td/6; %ideal time taken by agent to reach its goal without conflicts
end

MyVideo = VideoWriter('ConeVideo');
MyVideo.FrameRate = 10;
open(MyVideo);
    

avoidance_time = [];
kk = 1;
    
for icount=0:t_step:10000
    m = 1;
    [bool, avoidance,t_comp, d_min, a, a_lo, a_la] = goal_reached(agents,t_step,tol_d,n,t_run,R);
    
    if is_test_req
        if isSimulation_req
            hold on;
            draw_positions(agents, goal, xmax, ymax, t_step);
        end
    end

    t_run = t_run + t_step;
  
    
%     disp(strcat('------------------------------- ' ,string(t_run) ,' -----------------------------------'));

    frame = getframe(gcf);
    writeVideo(MyVideo, frame);
    
    avoidance_time(kk,:) = avoidance(1,:);
    
    t_c1(kk,:) = t_comp;
            
    d_min1(kk,:) = d_min;

    % check for collision
    while(m<=n)
        if numel(agents(m).NeighbourDistance)==n-1
            p = 1;
            while(p<=n-1)
                if p ~= m
                    if norm(agents(p).Position - agents(m).Position) < 2*(agents(m).Radius)
                        flag = flag + 1; 
                    end
                end
                p = p+1;
            end
        end
        m = m+1;
    end
    
   if flag>1
    failure_count  = 1;
   end
    
    
    if isplot_req
       for s = 1:n
           x_position_info(kk,s) = agents(s).Position(1);
           y_position_info(kk,s) = agents(s).Position(2);
       end
    end
    
        for s = 1:n
           x_velocity_info(kk,s) = agents(s).NewVelocity(1);
           y_velocity_info(kk,s) = agents(s).NewVelocity(2);
           xv(kk,s) = x_velocity_info(kk,s);
           yv(kk,s) = y_velocity_info(kk,s);
           v(kk,s) = norm([xv(kk,s), yv(kk,s)]);
           velocity_info(kk,s) = v(kk,s);
           
            acceler(kk,s) = a(1,s);
            acc(kk,s) = sqrt(a(1,s));
  
            if isplot_req
                acc_long(kk,s) = a_lo(1,s); 
                acc_lat(kk,s) = a_la(1,s);
            else
                acc_long(kk,s) = a_lo(1,s)^2; 
                acc_lat(kk,s) = a_la(1,s)^2;
            end

            distance_t = [agents(s).Position; agents(s).Target];
             
            if pdist(distance_t) <= tol_d
                t_actual(kk,agents(s).Identity) = t_run;
            end
        end

    if bool == 0
        break;
    end
  
    kk = kk + 1;
%     clf

if t_run>350
   start
   goal
   velocities

   for t_run_count = 1:n
        distance_t_run_count = [agents(t_run_count).Position; agents(t_run_count).Target];
        if pdist(distance_t_run_count) > tol_d
            t_actual(kk,agents(t_run_count).Identity) = 350;
        end
   end
   failure_count  = 1;
   break;
end
  if icount == 4.4
        1;
  end
end
close(MyVideo);



for r = 1:n    
    
    t_c2 = t_c1(:,r); 
    t_c2(all(~t_c2,2),:) = [];  %matrix of computational time for all agents till they reach there goal point
    t_com(r) = mean(t_c2); %array of average computational time for each agent  
    
    if(r>size(t_actual,2))
        disp(strcat(int2str(r), int2str(size(t_actual,2))));
    end
    
    t_act = t_actual(:,r);
    t_act(all(~t_act,2),:) = []; 
    t_a(1,r) = t_act(1,1); %atual time taken by agent to reach its goal with conflict situations
    eta(1,r) = t_d(r)/t_act(1,1); %efficieny t_ideal/t_actual;
    
    acceleration = acceler(:,r); 
    real_acc = isreal(acceleration);
    if real_acc == 0
        1;
    end
    acceleration = rmmissing(acceleration);
    acceleration(all(~acceleration,2),:) = [];
    a_rs(1,r) = sqrt(sum(acceleration));
    if numel(acceleration) == 0
        a_rms(1,r) = 0;
    else
        a_rms(1,r) = a_rs(1,r)/size(acceleration,1);
    end

    acceler_long = acc_long(:,r);
    real_acc_long = isreal(acceler_long);
    if real_acc_long == 0
        1;
    end
    acceler_long = rmmissing(acceler_long);
    acceler_long(all(~acceler_long,2),:) = [];
    acc_lo(1,r) = sqrt(sum(acceler_long));
    if numel(acceler_long) == 0
        acc_lo_rms(1,r) = 0;
    else
        acc_lo_rms(1,r) = acc_lo(1,r)/size(acceler_long,1);
    end

    acceler_lat = acc_lat(:,r);
    real_acc_lat = isreal(acceler_lat);
    if real_acc_lat == 0
        1;
    end
    acceler_lat = rmmissing(acceler_lat);
    acceler_lat(all(~acceler_lat,2),:) = [];
    acc_la(1,r) = sqrt(sum(acceler_lat));
    if numel(acceler_lat) == 0
        acc_la_rms(1,r) = 0;
    else
        acc_la_rms(1,r) = acc_la(1,r)/size(acceler_lat,1);
    end

    
end

ti = 0:t_step:icount;
if isplot_req
    
    figure(1)
    plot(ti,acc_long(:,1),Marker='.');
    hold on;
    plot(ti,acc_long(:,2),Marker='x');
    hold on;
    plot(ti,acc_long(:,3),Marker='+');
    hold on;
    plot(ti,acc_long(:,4),Marker='*');
    hold on;
    plot(ti,acc_long(:,5),Marker='o');
    
    figure(2)
    
    plot(ti,acc_lat(:,1),Marker='.');
    hold on;
    plot(ti,acc_lat(:,2),Marker='x');
    hold on;
    plot(ti,acc_lat(:,3),Marker='+');
    hold on;
    plot(ti,acc_lat(:,4),Marker='*');
    hold on;
    plot(ti,acc_lat(:,5),Marker='o');
    
    figure(3)
    plot(ti,acc(:,1),Marker='.');
    hold on;
    plot(ti,acc(:,2),Marker='x');
    hold on;
    plot(ti,acc(:,3),Marker='+');
    hold on;
    plot(ti,acc(:,4),Marker='*');
    hold on;
    plot(ti,acc(:,5),Marker='o');
end 
d_minimum = min(d_min1); %array of minimum distance for each agent
t_mission = mean(t_a);
eta_min = min(eta);
eta_mean = mean(eta);
a_rms_mean = mean(a_rms);
a_long_mean = mean(acc_lo_rms);
a_lat_mean = mean(acc_la_rms);

if flag>2
    start
    goal
    velocities
end

if is_test_req
    disp(strcat('collision happend --> ', int2str(flag/2), ' times'));
end
disp(strcat('collision happend --> ', int2str(flag/2), ' times'));
disp(strcat('Machine time --> ',int2str(toc(machine))));
disp(strcat('------------------------------- Mission Time ' , string(t_run) ,' -----------------------------------'));


% ti = 0:t_step:icount;
% figure(4)
% for k = 1:n
%    subplot(n,1,k);
%    plot(ti, avoidance_time(:,k)); % Avoidance vs Time plot
% end
%  1;
% % 
% ScenarioColours = [];
% iFontSize = 14;
% txtFontStyle = 'times';
% 
% 
% for PursuerNo=1:1:n
%     ScenarioColours = [ScenarioColours; GetColour(PursuerNo)]; %#ok<AGROW> 
%     legend_text = strcat('A_',string(PursuerNo));
% end
% 
% x_position_info(all(~x_position_info,2),:) = [];
% y_position_info(all(~y_position_info,2),:) = [];
% % 
% 
% if isplot_req                      % Position plot
%     figure(5)  
%     for s = 1:n
%        
%        hold on;
%        
%        legend_text = strcat('Agent A_',string(s));
%        plothandle(s) = plot(x_position_info(1:end,s),y_position_info(1:end,s), 'LineWidth',1.5, 'Color',ScenarioColours(s,:)); %#ok<AGROW> 
%        plot(goal(s,1),goal(s,2), 'Color',ScenarioColours(s,:), 'Marker','o', MarkerSize=7, LineWidth=3);
%        plot(x_position_info(1:1,s),y_position_info(1:1,s), 'Color',ScenarioColours(s,:), 'Marker', 'diamond', MarkerSize=7, LineWidth=3);
%    end
% end
% % % legend([plothandle(1), plothandle(2)],{'Agent A1','Agent A2'})
% legend([plothandle(1), plothandle(2), plothandle(3), plothandle(4), plothandle(5)],{'A_1','A_2','A_3', 'A_4', 'A_5'})
% axis ('equal');
% axis([-1 61 0 66]);
% xlabel('Position in X, m');
% ylabel('Position in Y, m');
% % legend('Pursuer''s trajectory','Target''s trajectory');
% grid on;
% pbaspect([1 1 1])
% legend
% axis normal;
% 1;
% ax = gca;
% set(ax,'fontsize',iFontSize,'fontweight','normal');
% set(ax,'fontname',txtFontStyle)  % Set fontstyle to times
% box(ax,'on');
% ax.GridColor = [0, 0, 0];  % [R, G, B]
% ax.GridAlpha = 1;
% ax.GridLineStyle = ':';
% 1;
% % % 
% 
% if isplot_req
%     figure
%     subplot(2,1,1);
%       for s = 1:n
%         plothandle(s) = plot(ti(:), acc_long(:,s),'LineWidth',1, 'Color',ScenarioColours(s,:));
%         hold on;
%       end
%     subplot(2,1,2);
%       for s = 1:n
%         plothandle(s) = plot(ti(:), acc_lat(:,s),'LineWidth',1, 'Color',ScenarioColours(s,:));
%         hold on;
%       end
% end



% velocity_info(all(~velocity_info,2),:) = []; 
% figure
% if isplot_req                                  % Velocity vs Time plot
%    for s = 1:n
%        plothandle(s) = plot(ti(:), velocity_info(:,s),'LineWidth',1.5, 'Color',ScenarioColours(s,:));
%        hold on;
%    end
% end
% legend([plothandle(1), plothandle(2), plothandle(3), plothandle(4), plothandle(5)],{'A_1','A_2','A_3', 'A_4', 'A_5'})
% 
% acceler(kk+1:end, :) = [];
% figure
% if isplot_req                                    % Acceleration vs Time plot
%    for s = 1:n
%        plothandle(s) = plot(ti, acc(:,s),'LineWidth',1.5, 'Color',ScenarioColours(s,:));
%        hold on;
%    end
% end
% legend([plothandle(1), plothandle(2), plothandle(3), plothandle(4), plothandle(5)],{'A_1','A_2','A_3', 'A_4', 'A_5'})
% 
% x_velocity_info(all(~x_velocity_info,2),:) = []; 
% y_velocity_info(all(~y_velocity_info,2),:) = [];
% 
% figure
% quiver(x_position_info(1:end,:),y_position_info(1:end,:), x_velocity_info(:,:), y_velocity_info(:,:), 0.1);
% axis ('equal');
% axis([0 100 0 100]);
% 1;
end