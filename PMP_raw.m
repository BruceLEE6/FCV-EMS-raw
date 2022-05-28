%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PMP used for data challenge adapted for vehicle use
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; 
clc; 
close all;
load('Power_FCV_demande.mat')
% the original sample period is 0.005s. down sampling by 0.5/0.005 = 100 the
% control period is then 0.5s
Timestep=1/200*100; 
T_demand = P.time(1:100:end); 
P_demand = P.signals.values(1:100:end,1); % here the original data are multiplied by 3.5 in order to have an adaptive scale
% the overall look of the load power shape
figure
plot(T_demand,P_demand)
xlabel('Time [s]');
ylabel('Power [W]');
set(get(gca,'XLabel'),'FontSize',12);%
set(get(gca,'YLabel'),'FontSize',12);
set(get(gca,'ZLabel'),'FontSize',12);
set(gcf,'position',[232   246   560   320])
set(gca,'XGrid','on','YGrid','on','GridLineStyle',':');

% the length of the load power time series
L = length(P_demand);

% final value of SOC, it should not be a fix value, but be limited in a
% narrow interval
SOC_final_max = 70.1; 
SOC_final_min = 69.9;
SOC_initial = 70; % initial value of SOC

%%
% define the involved variables
U_opt = zeros(L,1); % power split ratio of SC
H_opt = zeros(L,1); % Hamiltonian function
C_opt = zeros(L,1); % Cost or hydrogen consuming
SOC_opt = zeros(L+1,1); % SOC
lambda_opt = zeros(L+1,1); % lambda 

SOC_opt(1) = SOC_initial; % the trajetory of SOC is always begun by the same value
%here we use shooting method to find the proper value of inp.lambda{1}
n_gen = 0; % first generation to optimize the inp.lambda{1}
lambda_sup = -1; % the up limit of the searching interval
lambda_inf = -5; % the down limit of the searching interval
Net_a = [0:10:400]'; % action discrete vector. action variable: fuel cell current

%%
while(1)
% begin iteration
n_gen = n_gen +1; % index of iteration
lambda_initial = (lambda_sup + lambda_inf)/2 % update lambda_0-2.125-2.0938%
SOC_opt(1) = SOC_initial; % initialize SOC
lambda_opt(1) = lambda_initial; % initialize lambda
    for k = 1:L % at each time step, do instantous optimization
        [cost_temp, state_out_temp, lambda_out_temp, H_temp, output_temp] = func_pmp_raw(Net_a, SOC_opt(k), lambda_opt(k), P_demand(k)); %[X C H I out] = fcev_pmp(inp,par)
        [H_opt(k), index_temp] = min(H_temp);
        U_opt(k) = Net_a(index_temp); % power split ratio of SC
        C_opt(k) = cost_temp(index_temp); % Cost or hydrogen consuming
        SOC_opt(k+1) = state_out_temp(index_temp); % save current optimal SOC
        lambda_opt(k+1) = lambda_out_temp(index_temp); % save current optimal lambda
    end
disp('SOC end')
SOC_opt(end) % display final SOC

if SOC_opt(end)<SOC_final_min
    lambda_sup = lambda_initial;
    lambda_initial = (lambda_sup + lambda_inf)/2;
elseif SOC_opt(end)>SOC_final_max 
    lambda_inf = lambda_initial;
    lambda_initial = (lambda_sup + lambda_inf)/2;
else
    lambda_save = lambda_initial;
    break % if the final SOC satisfies the limit, break the while loop 
end
end
% display the total consumption of hydrogen.
disp('consumption')
sum(C_opt)
figure 
plot(lambda_opt)
title('Optimal co-state trajectory')
figure 
plot(SOC_opt)
title('Optimal SOC trajectory')
figure 
plot(U_opt)
title('Optimal action sequence')
figure 
plot(C_opt)
title('Optimal cost sequence')