%% Function describing the cost related to the final state. 
% set: the final SOC. Only constrain the down limit. 
function cost = func_phi_raw(state_in, set)
cost = 10*max(set - state_in,0);
