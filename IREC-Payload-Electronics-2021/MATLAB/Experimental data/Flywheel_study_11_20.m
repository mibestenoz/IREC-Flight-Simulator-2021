% FLYWHEEL STUDY
% Created by: Wiler Sanchez
% Started: 11/20/2021
% Updated: -
%--------------------------------------------------------------------------
% PURPOSE
%   - Determine the right motor to spin our payload
%--------------------------------------------------------------------------
% SOURCE
%   https://mechguru.com/machine-design/flywheel-design-calculation-simplified/
%--------------------------------------------------------------------------
clc; clear;
%--------------------------------------------------------------------------
% Experimental data 
%--------------------------------------------------------------------------
I = 0.0097536796;       % kg*m^2
test1 = table2array(importfile('payload roll test 1.csv'));     % rad/s
test2 = table2array(importfile('payload roll test 2.csv'));
test3 = table2array(importfile('payload roll test 3.csv'));
test4 = table2array(importfile('payload spin test 1.csv'));
time1 = linspace(0,length(test1)*.1,length(test1))';
time2 = linspace(0,length(test2)*.1,length(test2))';
time3 = linspace(0,length(test3)*.1,length(test3))';
time4 = linspace(0,length(test4)*.1,length(test4))';
ed1 = [time1,test1]; ed2 = [time2,test2];
ed3 = [time3,test3]; ed4 = [time4,test4];

%--------------------------------------------------------------------------
% Extracting important parameters
%--------------------------------------------------------------------------
wmax = zeros(4,1); wmin = zeros(4,1);
wmax(1) = max(ed1(:,2)); wmin(1) = min(ed1(:,2));
wmax(2) = max(ed2(:,2)); wmin(2) = min(ed2(:,2));
wmax(3) = max(ed3(:,2)); wmin(3) = min(ed3(:,2));
wmax(4) = max(ed4(:,2)); wmin(4) = min(ed4(:,2));

%--------------------------------------------------------------------------
% Differentiate angular velocity
%--------------------------------------------------------------------------
alpha1 = (diff(test1)./diff(time1));
alpha2 = (diff(test2)./diff(time2));
alpha3 = (diff(test3)./diff(time3));
alpha4 = (diff(test4)./diff(time4));

%--------------------------------------------------------------------------
% Calculations
% The smasller the Cs value, the larger the flywheel, but smoother
% operation.
%--------------------------------------------------------------------------

Cs = zeros(4,1);        % Coefficient of fluctuation of speed
wmean = zeros(4,1);     % Mean angular velocity
Ke = zeros(4,1);        % Kinetic energy
for i = 1:4
Cs(i) = (2*(wmax(i)-wmin(i)))/(wmax(i)+wmin(i));
wmean(i) = (wmax(i)-wmin(i))/2;
Ke(i) = 0.5*I*((wmax(i)^2)-(wmin(i)^2));
end 
