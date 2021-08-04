% gust simulator script 
clc; clear; close all; tic

global turbulence_intensity; 
global max_acceleration;    %declare max acceleration
global m_empty;             %declare unloaded mass of rocket
global rail_length;         %declare rail length
global g;                   %declare gravitational acceleration
global gamma;               %declare ratio of specific heats
global R;                   %declare gas constant
global T_ground;            %declare temperature on launch pad
global lapse_rate;          %declare troposphere lapse rate
global main_altitude;       %declare main parachute deployment altitude

main_altitude = 198;        %main parachute deployment altitude (m)
max_acceleration = 0;       %initialize max acceleration (m/s^2)
m_empty = 15.2171;          %unloaded mass of rocket (kg)
m_propellant = 2.066;       %mass of propellant (kg)
m_aft = 4.61;               %mass of aft section (heaviest section) (kg)
rail_length = 8;            %launch rail length (ft)
g = 9.81;                   %gravitational acceleration (m/s^2)
gamma = 1.4;                %ratio of specific heats
R = 287;                    %gas constant (J/kg*K)
T_ground = 292;             %temperature on launch pad (K)
lapse_rate = 0.0065;        %troposphere lapse rate (K/m)

%Load time (s) and thrust (N) data
global time_data;
time_data = csvread('Simulation Thrust.csv',0,0,[0 0 97 0]);
global thrust_data;
thrust_data = csvread('Simulation Thrust.csv',0,1,[0 1 97 1]);

%
% Define variables for velocity of wind
%
global windupper;
global windlower;

%
% convert to fixed time step solver
%
t0 = 0;
tf = 100;
nsteps = 1000;
tspan = linspace(t0,tf,nsteps);

%
% Initialize upper and lower wind values for bandwidth
%
windupper = 5; % m/s
windlower = -5;

%
% Initialize turbulence intensity
% Choose from: 
% light = 1
% moderate = 2
% severe = 3
%
turbulence_intensity = 1;

%
% Amount of simulations
%
ii = 100;

apogee = zeros(ii,1);
for i = 1:ii
    [t,y] = ode45(@differentialEquation,tspan,[0;0;0;0;m_propellant;0]);
    
    %Remove data after landing
    for ii = 1:length(t)
        if y(ii,3) < 0 && y(ii,4) < 0 
            break;
        end
    end
    apogee(i) = max(y(:,3));
end
apogee(0 == apogee) = nan;
figure(1)
edges = min(apogee):1:max(apogee);
histogram(apogee,edges);
if turbulence_intensity == 1; turb = 'light'; 
elseif turbulence_intensity == 2; turb = 'moderate';
elseif turbulence_intensity == 3; turb = 'severe'; 
end

title('Apogee Histogram Turbulence Intensity:',turb);
ylabel('# of Times Apogee Reached'); xlabel('Apogee [m]');
toc