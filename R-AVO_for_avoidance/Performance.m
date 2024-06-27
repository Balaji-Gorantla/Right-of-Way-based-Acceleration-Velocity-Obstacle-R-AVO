clear all;close all;clc;

global n;

dmin = [];
t_c1 = [];
F_c = 0;
F_c1 = 0;


NumAgents = [10, 20, 30, 40];

NoOfDifferentAgentTrails = size(NumAgents,2);
NoOfDifferentInitTrials = 10;

t_com1 = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
d_min = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
eta_minimum = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
eta_mean = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
a_rms_mean = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
t_m = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
a_long_rms_mean = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);
a_lat_rms_mean = nan(NoOfDifferentInitTrials,NoOfDifferentAgentTrails);

for iCount = 1:1:NoOfDifferentAgentTrails
     
    n = NumAgents(iCount);
    failure_count(1,iCount) = 0;
    
    for j = 1:1:NoOfDifferentInitTrials
        disp(strcat(string(iCount), '-->' , string(j)));
        [t_mission,t_com,d_minimum, eta_min, eta_avg, a_rms, failure, a_long_rms, a_lat_rms] = main(n);
        
        t_m(j,iCount) = t_mission;
        t_com1(j,iCount) = mean(t_com);
        d_min(j,iCount) = min(d_minimum);
        eta_minimum(j,iCount) = eta_min;
        eta_mean(j,iCount) = eta_avg;
        a_rms = rmmissing(a_rms);
        a_rms_mean(j,iCount) = mean(a_rms);
        a_long_rms = rmmissing(a_long_rms);
        a_lat_rms = rmmissing(a_lat_rms);
        a_long_rms_mean(j,iCount) = mean(a_long_rms);
        a_lat_rms_mean(j,iCount) = mean(a_lat_rms);
        failure_count(1,iCount) = failure_count(1,iCount)+failure;
        
        
%         if(numel(t_c1) > 0)
%             if(size(t_c1,1) > size(t_com1,1))
%                 t_com1 = [t_com1;nan(size(t_c1,1) - size(t_com1,1),1)];
%             elseif(size(t_com1,1) > size(t_c1,1) )
%                 t_c1 = [t_c1;nan(size(t_com1,1) - size(t_c1,1),1)];
%             end
%         end
%         
%         t_c1(j) = [t_c1, t_com1];

        disp(strcat('Failures happened-->  ', int2str(failure_count), '/', int2str(j)));
%         F_c = F_c + failure_count;
%         if NoOfDifferentAgentTrails > 1
%             F_c1 = F_c1 + F_c + failure_count;
%             disp(strcat('Failures happened ', int2str(F_c1), '/', int2str(j+NoOfDifferentInitTrials)));
%         end
    end
%     dmin = [dmin, d_min'];
    
end

figure(1)
boxplot(d_min,'Labels',{'K = 10','K = 20', 'K = 30', ' K = 40'});
xlabel('No. of Agents');
ylabel('Minimum distance');
savefig("d_min.fig");

figure(2)
boxplot(t_com1,'Labels',{'K = 10','K = 20', 'K = 30', ' K = 40'});
xlabel('No. of Agents');
ylabel('computional time');
savefig("t_comp");


% figure(3)
% boxplot(eta_minimum, 'Labels',{'K = 8','K = 15','K = 25'});
% xlabel('No. of Agents');
% ylabel('Efficiency(Minimum)');
% savefig("eta_min");
% 
% figure(4)
% boxplot(eta_mean,'Labels',{'K = 8','K = 15','K = 25'});    %{'K = 6','K = 8','K = 10'};
% xlabel('No. of Agents');
% ylabel('Efficiency(Mean)');
% savefig("eta_mean");


figure(5)
boxplot(a_rms_mean,'Labels',{'K = 10','K = 20', 'K = 30', ' K = 40'});%{'K = 6','K = 8','K = 10'});
xlabel('No. of Agents');
ylabel('RMS of Acceleration');
savefig("a_rms");

figure(6)
boxplot(t_m,'Labels',{'K = 10','K = 20', 'K = 30', ' K = 40'});%{'K = 6','K = 8','K = 10'});
xlabel('No. of Agents');
ylabel('Mission Time');
savefig("t_mission");

figure(7)
iCount = 1:1:NoOfDifferentAgentTrails;
plot(iCount,failure_count(1,:));
xlabel('No. of Agents');
ylabel('No. of Failures');
savefig('Failure_Count');
% 

figure(8)
boxplot(a_lat_rms_mean,'Labels',{'K = 10','K = 20', 'K = 30', ' K = 40'});%{'K = 6','K = 8','K = 10'});
xlabel('No. of Agents');
ylabel('Lateral Acceleration');
savefig("a_lat_rms");

figure(9)
boxplot(a_long_rms_mean,'Labels',{'K = 10','K = 20', 'K = 30', ' K = 40'});%{'K = 6','K = 8','K = 10'});
xlabel('No. of Agents');
ylabel('Longitudinal Acceleration');
savefig("a_long_rms");

for k = 1:1:NoOfDifferentAgentTrails
t_m_avg(1,k) = mean(t_m(:,k));
t_c_avg(1,k) = mean(t_com1(:,k));
d_min_avg(1,k) = mean(d_min(:,k));
eta_minimum_avg(1,k) = mean(eta_minimum(:,k));
eta_mean_avg(1,k) = mean(eta_mean(:,k));
acc_rms_avg(1,k) = mean(a_rms_mean(:,k));
 1;
end

