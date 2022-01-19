function [cost, state_out, output] = func_L_raw(action, state_in, para)
Timestep = 0.5; % the time step should be set according to the nature of the problem. In our case, 0.5 s should be small enough.
%% Model parameters of the 16 kW Fuel Cell (Prof. Loic Boulon) 
% here are the parameters related to empirical polarization V-I function. 
a2=0;
b2=-6.7791e-07;
c2=0.00044927;
d2=-0.11913;
e2=59.124;
% hydrogen consumption coefficients. 
g2=0.4374*1.2;
h2=15.835;

r_L = 0.0049;% resistance of inductance
eita_boost = 0.95;
% voltage of fc output
% i_fc = [0:1:400];
% u_fc = 0*power(i_fc,4)+b2*power(i_fc,3)+c2*power(i_fc,2)+d2*i_fc+e2;
% figure
% plot(i_fc,u_fc)
% h2 consumption
% h2ps = (g2*i_fc+h2)*0.08988/60;
% figure
% plot(i_fc,h2ps)

%P_fc_out = (i_fc*u_fc-i_fc^2*r_L)*eita_boost;

%% Li-ion battery ()
BAT.Q_bat = 40;             % storage capacity (Ah)
BAT.U_cel_nom = 3.35;       % nominal cell voltage (V)
BAT.U_cel_min = 2.5;        % minimal cell voltage (V)
BAT.U_cel_max = 4.2;        % maximal cell voltage (V)
BAT.Nbre_cel_serie = 6;     % number of cells in series for a module
BAT.Nbre_mod_serie = 4;     % number of modules in series 
BAT.U_nom = BAT.U_cel_nom*BAT.Nbre_cel_serie*BAT.Nbre_mod_serie;  % nominal battery voltage (V)
BAT.U_min = BAT.U_cel_min*BAT.Nbre_cel_serie*BAT.Nbre_mod_serie;  % minimal battery voltage (V)
BAT.U_max = BAT.U_cel_max*BAT.Nbre_cel_serie*BAT.Nbre_mod_serie;  % maximal battery voltage (V)

BAT.Ceq = BAT.Q_bat*3600/(BAT.U_max-BAT.U_min);   % equivalent capacity of the battery
r_bat=0.028;                                      % internal resistance of the battery
% 1-order battery model is considered in our case, so the parallel resistor
% is ignored while calculating battery current and SOC variation
rc=500/BAT.Ceq;                                   % parallel resistor 


%% DP implementation

Num_state = size(state_in,1);
Num_action = size(action,1); 
p_demand  = para;% load power

% 
SOC_min=30;                                              % turn ON of the FC
SOC_max=75;                                              % turn OFF of the FC

U0 = interp1([20 90]',[BAT.U_nom*0.95 BAT.U_nom*1.05]',state_in,'linear','extrap');
u_fc = 0*power(action,4)+b2*power(action,3)+c2*power(action,2)+d2*action+e2;
P_fc = (action.*u_fc-action.^2*r_L)*eita_boost;

P_bat = p_demand - P_fc'; % notice that here the transition operation makes P_fc a row vector.

% battery output current is calculated using the battery power and
% parameters
i_bat =  0.5*(U0-sqrt(U0.^2-4*P_bat*r_bat))/r_bat;
% To handle the simutation that delta < 0
i_bat = (conj(i_bat)+i_bat)/2;

state_out = state_in - 100/3600/BAT.Q_bat*(i_bat.*Timestep); % state_out is a matrix with the Num_state rows, Num_action columns.
cost = ones(Num_state,1)*(g2*action+h2*ones(size(action)))'*Timestep*0.08988/60; % cost is a matrix with the Num_state rows, Num_action columns.
cost = cost + 5*max(max(state_out - SOC_max,SOC_min - state_out),0); % additional costs should be added when the limits of SOC are violated 

% the output variables are only used for debugging
output.u_fc = u_fc;
output.P_fc = P_fc;
output.P_bat = P_bat;
output.i_bat = i_bat;
