%rocket stuff
clc;clear;
L = 1234.07; %N
D = 575; %N

%% Flight parameters 
a_total = ; %Total acceleration
m = ; %total mass 
q = ; %dynamic pressure
c = ; %chord length

%%
%Moment by rocket
xcp = 2.645; %m at burnout
xcg = 2.212; %m at burnout
x_acs = 90.2e-2; %m fixed
alpha = 3 * pi/180; %rad
M = -(xcp - xcg) * (L*cos(alpha) + D*sin(alpha));
F_ACS = M / (x_acs-xcg);
S = m*a_total/(q*c*(L-D));
