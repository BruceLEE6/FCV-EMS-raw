%% Function describing the cost related to the final state. 
% set: the final SOC. Only constrain the down limit. 
% here we add additional cost only when the final SOC is less than the
% initial value. The cost is proportional to the difference between the
% initial value and the final value. The coefficient can be regulated in
% this view. 
function cost = func_phi_raw(state_in, set)
cost = 10*max(set - state_in,0);
