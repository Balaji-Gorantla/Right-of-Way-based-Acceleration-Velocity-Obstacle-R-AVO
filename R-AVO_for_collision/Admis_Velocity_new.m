function L = Admis_Velocity_new(obj,time) 
    
Q = [];
P = [];
global is_test_req;
global v_max;

vel_x =linspace(-v_max,v_max,10);% + obj.Position(1);
vel_y =linspace(-v_max,v_max,10);% + obj.Position(2);
[vel_optx, vel_opty]=meshgrid(vel_x, vel_y);
P = [vel_optx(:), vel_opty(:)];
alpha_agent = atan2(obj.NewVelocity(2),obj.NewVelocity(1));
rotation_matrix = [cos(alpha_agent), -sin(alpha_agent); sin(alpha_agent),cos(alpha_agent)];
A_P = rotation_matrix*P';
P = A_P';
P = [P(:,1) + obj.Position(1), P(:,2) + obj.Position(2)];
% P = [P(:,1), P(:,2)];

v_pref = obj.Position + obj.NewVelocity*time;
theta_goal = atan2((obj.Target(2)-obj.Position(2)), (obj.Target(1)-obj.Position(1))); 
v_goal = [cos(theta_goal), sin(theta_goal)] + [obj.Position(1), obj.Position(2)];


for k = 1:numel(P)/2
    is_right = right_left(P(k,:),obj.Position,v_pref);
    Q(k) = is_right;
end

% Z = [P(4,:),Q(4)];
% 
% if Q(4) == -1
%     1;
% end

Z = [ P,Q'];

% hold on;
% scatter(P(:,1),P(:,2));
% scatter(v_pref(1,1),v_pref(1,2),'k*');
% hold off;
%for right-of-way(RoW) (only move from right side)

[a,~]=size(Z);
for i = 1:a
%     if Z(i,3) == -1
%         Z(i,:) = [0,0,0];
%     else
        D=[Z(i,1:2); v_pref ];
%         dist = pdist(D);
%         Z(i,3) = dist;  
        min_goal = atan2((v_goal(2)-Z(i,2)),(v_goal(1)-Z(i,1)));
        Z(i,3) = min_goal;
%     end
end

Z(all(~Z,2),:) = [];

%all admissible velocities(no RoW)

% [a,b]=size(Z);
% for i = 1:a
%     D=[Z(i,1:2); v_pref ];
%     dist = pdist(D);
%     Z(i,3) = dist; 
% end
% 
% hold on;
% scatter(Z(:,1),Z(:,2));
% scatter(v_pref(1,1),v_pref(1,2),'r*');
% hold off;

Z(:,1) = Z(:,1) - obj.Position(1);
Z(:,2) = Z(:,2) - obj.Position(2);

L = [];
for kCount = 1:size(Z,1)
   [~, index] = max(Z(:,3)); %sorts velolcities for minimum deviation from goal diretion in ascending order
   L(kCount,1:3) = Z(index,1:3);
   Z(index,:) = [];
end
%   if numel(L) == 0
%        print_values;
%        1;
%        L = P(4,:);
%        L
%   end
L =L(1:end,1:2);

if is_test_req
    if obj.Identity == 1
%         hold on;
        scatter(L(:,1)+obj.Position(1),L(:,2)+obj.Position(2));
%         hold off;
    end
end

end