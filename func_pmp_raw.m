function [cost, state_out, lambda_out, H, output] = func_pmp_raw(action, state_in, lambda_in, para) %[X C H I out] = fcev_pmp(inp,par)
%% Model parameters of the 16 kW Fuel Cell (Prof. Loic Boulon) 
Timestep=1/200*100; 
a2=0;
b2=-6.7791e-07;
c2=0.00044927;
d2=-0.11913;
e2=59.124;

g2=0.4374*1.2;
h2=15.835;

r_L = 0.0049;% resistance of inductance
eita_boost = 0.95;

%% Li-ion battery
BAT.Q_bat = 40;             % storage capacity (Ah)
BAT.U_cel_nom = 3.35;       % nominal cell voltage (V)
BAT.U_cel_min = 2.5;        % minimal cell voltage (V)
BAT.U_cel_max = 4.2;        % maximal cell voltage (V)
BAT.m_cel = 5.6;            % cell weight (kg)
BAT.Nbre_cel_serie = 6;     % number of cells in series for a module
BAT.Nbre_mod_serie = 4;     % number of modules in series 
Mbat=BAT.m_cel*6;           % equivalent weight (1/4 of the original Tazzari Zero batteries) 

BAT.U_nom = BAT.U_cel_nom*BAT.Nbre_cel_serie*BAT.Nbre_mod_serie;  % nominal battery voltage (V)
BAT.U_min = BAT.U_cel_min*BAT.Nbre_cel_serie*BAT.Nbre_mod_serie;  % minimal battery voltage (V)
BAT.U_max = BAT.U_cel_max*BAT.Nbre_cel_serie*BAT.Nbre_mod_serie;  % maximal battery voltage (V)
SOC0 = 70;                  % initial state of charge of the battery (%) 70
warning_limit=0;            % the vehicle can't move at this state of charge

BAT.Ceq = BAT.Q_bat*3600/(BAT.U_max-BAT.U_min);   % equivalent capacity of the battery
r_bat=0.028;                                      % internal resistance of the battery
rc=500/BAT.Ceq;                                   % parallel resistor
%%
% 
SOC_min=30;                                              % turn ON of the FC
SOC_max=90;                                              % turn OFF of the FC

K = 1; 
omega = K*(state_in>SOC_max)-K*(state_in<SOC_min); % penalty function

Num_state = size(state_in,1);
Num_action = size(action,1); 
lambda = lambda_in; % current value of lambda
p_demand  = para; % current load power

i_fc  =  action;
SOC = state_in;
U0 = interp1([20 90]',[BAT.U_nom*0.95 BAT.U_nom*1.05]',SOC,'linear','extrap');
u_fc = 0*power(i_fc,4)+b2*power(i_fc,3)+c2*power(i_fc,2)+d2*i_fc+e2; % this is the option to use the Loic model
P_fc = (i_fc.*u_fc-i_fc.^2*r_L)*eita_boost;

P_bat = p_demand - P_fc'; % notice that here the transition operation makes P_fc a row vector.
% battery output current is calculated using the battery power and
% parameters
i_bat =  0.5*(U0-sqrt(U0.^2-4*P_bat*r_bat))/r_bat;
% To handle the simutation that delta < 0
i_bat = (conj(i_bat)+i_bat)/2;

state_out = SOC - 100/3600/BAT.Q_bat*i_bat*Timestep;
cost = ones(Num_state,1)*(g2*action+h2*ones(size(action)))'*Timestep*0.08988/60; % cost is a matrix with the Num_state rows, Num_action columns.

%C{1} = (g2*i_fc+h2)*0.08988/60;%(g2*i_fc+h2)*0.08988/60*stepsize;
H = cost - (lambda_in+omega)*ones(1,Num_action).*i_bat*100/3600/BAT.Q_bat ; % Hamiltonian function
% derivative lambda
lambda_out = lambda_in*ones(1,Num_action) + Timestep*(lambda+omega)*ones(1,Num_action)*100/3600/BAT.Q_bat*0.5.*(1-U0*ones(1,Num_action)./sqrt(U0.^2-4*P_bat*r_bat))/r_bat*0.1*BAT.U_nom/70;% new lambda
output = action;