clear all; 
clc; 
close all;
% load driving cycle
load('Power_FCV_demande.mat')
% the original sample period is 0.005s. down sampling with by 100 the
% control period is then 0.5s
T_demand = P.time(1:100:end); 
P_demand = P.signals.values(1:100:end,1); % here the original data are multiplied by 3.5 in order to have an adaptive scale
figure
plot(T_demand,P_demand)
xlabel('Time [s]');
ylabel('Power [W]');
set(get(gca,'XLabel'),'FontSize',12);%
set(get(gca,'YLabel'),'FontSize',12);
set(get(gca,'ZLabel'),'FontSize',12);
set(gcf,'position',[232   246   560   320])
set(gca,'XGrid','on','YGrid','on','GridLineStyle',':');
%P_demand = P_demand(1:end-1);
L = length(P_demand);

SOC_final_min = 70;
SOC_final_max = 75;
% create grid
clear grd
% grd.Nx{1}    = 46; % the resolution is 0.01
% grd.Xn{1}.hi = 75; % low limit of SOC is 1
% grd.Xn{1}.lo = 30; % low limit of SOC is 0.3
Net_s = [30:1:75]'; % FC current

Net_a = [0:10:400]'; % FC current
% the input here is the partition of the demand power on SC
% grd.Nu{1}    = 41;  % the resolution is 0.1
% grd.Un{1}.hi = 400;   % Att: Lower bound may vary with engine size.
% grd.Un{1}.lo = 0;	% Att: Lower bound may vary with engine size.

% set initial state
% grd.X0{1} = 70;

% final state constraints. here the final constraint should not be a fixed value.
% grd.XN{1}.hi = 75; 
% grd.XN{1}.lo = 70;

% define problem
% clear prb
% prb.W{1} = P_demand; % load power as the input
% prb.Ts = 1/200*100; % sample time
% prb.N = length(prb.W{1}); % number of points
% % set options
% options = dpm();
% % some set parameters, here these parameter are configured just as those in
% % the provided program.
% options.MyInf = 1000;
% options.BoundaryMethod = 'Line'; % also possible: 'none' or 'LevelSet';
% if strcmp(options.BoundaryMethod,'Line') 
%     %these options are only needed if 'Line' is used
%     options.Iter = 5;
%     options.Tol = 1e-8;
%     options.FixedGrid = 0;
% end
% [res dyn] = dpm(@fcv_dp_2019,[],grd,prb,options); % run the DP program
Num_state = size(Net_s,1);
C_t_g = zeros(L+1,Num_state);

state_initial = 70;
C_t_g(end,:) = fcv_dp_phi_raw(Net_s,state_initial);
for k = L:-1:1
    [cost_temp, state_out_temp, output_temp] = fcv_dp_raw(Net_a, Net_s, P_demand(k));
    C_t_g_temp = interp1(Net_s,C_t_g(k+1,:),state_out_temp,'spline');
    [C_t_g(k,:),index_a_temp] = min(cost_temp+C_t_g_temp,[],2);
end

%% foreward simulation
action_opt = zeros(L,1);
Cost_opt = zeros(L,1);
State_opt = zeros(L+1,1);
state_current = state_initial;
State_opt(1,:)= state_current;
for k = 1:L
    [cost_temp, state_out_temp, output_temp] = fcv_dp_raw(Net_a, State_opt(k), P_demand(k));
    C_t_g_temp = interp1(Net_s,C_t_g(k+1,:),state_out_temp);
    [C_t_fwd_temp,index_a_temp] = min(cost_temp+C_t_g_temp,[],2);
    State_opt(k+1) = state_out_temp(index_a_temp);
    action_opt(k) = Net_a(index_a_temp);
    Cost_opt(k) = cost_temp(index_a_temp);
end
%%
%--------------------------------------------------------------------------
% PLOT RESULTS:
%--------------------------------------------------------------------------
%Calculate Optimal Solution:
sum(Cost_opt)
figure
plot(State_opt)
title('optimal SOC')
figure
plot(action_opt)
title('optimal action')
figure
plot(Cost_opt)
title('optimal action')
