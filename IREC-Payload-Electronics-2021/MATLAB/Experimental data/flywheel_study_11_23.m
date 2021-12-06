clc;clear;close all
D = open('test_data.mat');
Time = D.Data(:,1); %ms
Time = Time*10^-3; %convert time to seconds
Angular_Velocity = D.Data(:,2); %deg/s
jj = 1;
count = 1;
for ii = 2 : length(Time)
    if Time(ii-1) > Time(ii)
        count = count + 1;
        jj = 1;
    end
    time(jj,count) = Time(ii);
    Ang_Vel_Sorted(jj,count) = Angular_Velocity(ii);
    jj = jj + 1;
end
Ang_Accel = (diff(Ang_Vel_Sorted(:,:))./diff(time(:,:)));
Ang_Vel_Sorted(1679,:) = [];
time(1679,:)= [];
for ii = 1:6
    figure()
    yyaxis left
    plot(time(:,ii),Ang_Vel_Sorted(:,ii))
    ylim([-220 220])
    xlabel('Time(s)')
    ylabel('Angular Velocity (deg/s)')
    hold on
    yyaxis right
    plot(time(:,ii),Ang_Accel(:,ii))
     ylim([-220 220])
    xlabel('Time(s)')
    ylabel('Angular Accel (deg/s^2)')
end