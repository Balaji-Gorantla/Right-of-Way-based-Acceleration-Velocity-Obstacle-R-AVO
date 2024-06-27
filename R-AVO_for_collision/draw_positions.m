function draw_positions(agents, goal, xmax, ymax, t_step)

l_a = length(agents);
a_p = [];
a_r = [];
for i = 1:l_a
   tar = []; 
   f  = hsv(l_a);
   
   color = f(i,:);
   a_p = [a_p; agents(i).Position];
   a_r = [a_r; agents(i).Radius];
  
   viscircles(agents(i).Position, agents(i).Radius*0.5, 'EdgeColor', color, 'LineWidth', 2); %draws the circle with center as position and radius of agents
   for j = 1:2
       tar=[tar,goal(i,j)];
   end
            
    target_Matrix(tar,color);
    tar=[];
  
end
% hold on;
% plot(a_p(:,1), a_p(:,2));
% hold off;
axis ('equal');
% axis([-100 200 -100 200]);
% axis([-30 100 40 80]);
axis([-20 120 -30 80]);

set(gcf, 'units','normalized','outerposition',[0 0 1 1]); %it sets the property values of the current figure(gcf: get current figure). 
                                                                   %setting normalized units,  the actual borders of the figure to be at the 
                                                                   %bottom left corner (0,0) and span the whole screen (1,1)

pause(0.01);


end