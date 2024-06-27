%% 
% input required
% 1. robo vel x and y
% 2. robo pos x and y amd its corner coordinate points in terms of x and y
% 3. the obstacles velocity and its x and y coordinates of its corner
% points

function [occupied_cells] = avo_cells_in_SO_linear(iter_obs_count,obstacle_matrix, pose, dia_agent,t_recur,t_num,Vopty,vby,Voptx,vbx,a_i_x,a_i_y,a_j_x,a_j_y)

t=t_recur(1,t_num);
xCenter=pose(1,1);% current position of the robot
yCenter=pose(2,1);
xCentero=obstacle_matrix(iter_obs_count,1);% current velocity of the obstacle
yCentero=obstacle_matrix(iter_obs_count,2);

width1 =   (sqrt(2)*dia_agent)/(t); % whatever
height1 =  (sqrt(2)*dia_agent)/(t);% whatever...
% xCenter1 = (delta*(exp(-t/delta)-1)*Vax-((xCenter-xCentero)-t*vbx))/((t+delta*(exp(-t/delta)-1)));% Wherever...
% yCenter1 = (delta*(exp(-t/delta)-1)*Vay-((yCenter-yCentero)-t*vby))/((t+delta*(exp(-t/delta)-1)));% Wherever..
 
xCenter1 = (xCenter-xCentero)/t + (a_i_x - a_j_x)*t;% Wherever...
yCenter1 = (yCenter-yCentero)/t + (a_i_y - a_j_y)*t;% Wherever..

x_min = xCenter1-width1/2;
y_min = yCenter1-height1/2;

x_max = xCenter1+width1/2;
y_max = yCenter1+height1/2;

if Voptx<=(vbx) + x_max && Voptx>=(vbx) + x_min && Vopty>=(vby) +y_min && Vopty<=(vby) +y_max 
    occupied_cells=1;
else
    occupied_cells=0;
end
end
