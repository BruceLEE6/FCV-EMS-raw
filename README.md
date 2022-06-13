# FCV-EMS-raw
This is the implementation of a raw dynamic programming (DP) for the energy management of a simple fuel cell electric vehicle (FCEV). 
## Models of the fuel cell electric vehicle
The model of the FCEV power sources and the cost functions are represented via the following two functions:
- func_L_raw.m: FCEV model function. 
- func_phi_raw.m: cost related to the final SOC.

The power demand of a specific driving cycle is saved in the following file:
- Power_FCV_demande.mat: load power versus time of an arbitrary driving cycle. 
## EMS methods
### Rule based methods
#### Basic rules

#### Fuzzy logic inference


### Optimization based methods
#### Dynamic programming
- DP_raw_live.mlx: The main DP program in live script matlab format to show intermediate results in a interactive way. 
- DP_raw.m: main DP program in .m format which is equivalent to the live script.
#### Pontryagin’s Minimum Principle
- func_pmp_raw.m: function used for PMP EMS
- PMP_raw.m: main program for PMP implementation
#### Equivalent Consumption Minimization Strategy
- func_ecms_raw.m: function used for ECMS EMS
- ECMS_raw.m main program for ECMS implementation
