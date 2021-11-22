clc; clear; close all;

global max_acceleration;%declare max acceleration
global m_empty;         %declare unloaded mass of rocket
global rail_length;     %declare rail length
global g;               %declare gravitational acceleration
global gamma;           %declare ratio of specific heats
global R;               %declare gas constant
global T_ground;        %declare temperature on launch pad
global lapse_rate;      %declare troposphere lapse rate
global main_altitude;   %declare main parachute deployment altitude

main_altitude = 198;    %main parachute deployment altitude (m)
max_acceleration = 0;   %initialize max acceleration (m/s^2)
m_empty = 16.521;       %unloaded mass of rocket (kg)
m_propellant = 8.987;   %mass of propellant (kg)
m_aft = 4.61;           %mass of aft section (heaviest section) (kg)
rail_length = 8;        %launch rail length (ft)
g = 9.81;               %gravitational acceleration (m/s^2)
gamma = 1.4;            %ratio of specific heats
R = 287;                %gas constant (J/kg*K)
T_ground = 292;         %temperature on launch pad (K)
lapse_rate = 0.0065;    %troposphere lapse rate (K/m)

%Load time (s) and thrust (N) data
simdata = readmatrix('AeroTech_M2000R.csv');
global time_data;
time_data = simdata(:,1);
global thrust_data;
thrust_data = simdata(:,2);

%Solve equations of motion
tspan = [0 100];
x0 = [0;0;0;0;m_propellant;0];
[t,y] = ode45(@differentialEquation,tspan,x0);

%Remove data after landing
for ii = 1:length(t)
    if y(ii,3) < 0 && y(ii,4) < 0 
        break;
    end
end
t = t(1:ii-1);
y = y(1:ii-1,:);

%Determine temperature throughout launch
T = T_ground - lapse_rate*y(:,3);

%Determine Mach number
M = y(:,4)./sqrt(gamma*R*T);

%Calculate ground hit kinetic energy (J)
KE = .5*m_aft*y(end,4)^2;

%Calculate Thrust-to-weight ratio
t2wr = mean(thrust_data)/((m_empty + m_propellant)*g);

%Convert to English units
y(:,1:4) = y(:,1:4)*3.28084;
max_acceleration = max_acceleration*3.28084;
main_altitude = main_altitude*3.28084;
KE = KE*0.737562;
thrust_data = thrust_data*0.224809;

%Determine recovery velocities (ft/s)
for ii = 1:length(t)
    if y(ii,4) < 0
        drogue_deployment_velocity = abs(y(ii+1,4));
        break;
    end
end
for ii = 1:length(t)
    if y(ii,4) < 0 && y(ii,3) <= main_altitude
        main_deployment_velocity = abs(y(ii-1,4));
        break;
    end
end
terminal_velocity = abs(y(end,4));

%Find mission performance predictions
[apogee, apogee_index] = max(y(:,3));
max_velocity = max(y(:,4));
max_Mach = max(M);
for jj = 1:length(t)
    if y(jj,3) >= rail_length 
        break;
    end
end
rail_exit_velocity = y(jj,4);
for kk = 1:length(t)
    if y(kk,3) > 0
        break;
    end
end
liftoff_thrust = interp1(time_data,thrust_data,t(kk));
drift_radius = abs(y(end,1));
descent_time = t(end) - t(apogee_index);

%Plot altitude
plot(t,y(:,3));
xlabel('Flight Time (s)');
ylabel('Altitude (ft)');

%Plot vertical velocity
figure();
plot(t,y(:,4));
xlabel('Flight Time (s)');
ylabel('Vertical Velocity (ft/s)');

%Plot thrust curve
figure();
plot(time_data,thrust_data);
xlabel('Flight Time (s)');
ylabel('Thrust (lbf)');

%Print mission performance predictions
fprintf('Apogee:\t\t\t\t\t\t\t\t\t%.0f ft\nMax Vertical Velocity:\t\t\t\t\t%.0f ft/s\nMax Vertical Acceleration:\t\t\t\t%.0f ft/s^2\nMax Mach Number:\t\t\t\t\t\t%.3f\nLiftoff Thrust:\t\t\t\t\t\t\t%.0f lbf\nThrust-to-Weight Ratio:\t\t\t\t\t%.2f\nRail Exit Velocity:\t\t\t\t\t\t%.0f ft/s\nMax Impact Kinetic Energy:\t\t\t\t%.0f ft*lbf\nDrift Radius:\t\t\t\t\t\t\t%.0f ft\nDescent Time:\t\t\t\t\t\t\t%.0f s\nDrogue Parachute Deployment Velocity:\t%.0f ft/s\nMain Parachute Deployment Velocity:\t\t%.0f ft/s\nTerminal Velocity:\t\t\t\t\t\t%.0f ft/s\n', apogee, max_velocity, max_acceleration, max_Mach, liftoff_thrust, t2wr, rail_exit_velocity, KE, drift_radius, descent_time, drogue_deployment_velocity, main_deployment_velocity, terminal_velocity);