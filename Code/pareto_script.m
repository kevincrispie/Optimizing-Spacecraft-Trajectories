clear all
close all

load Minfuel.mat
load Mintime.mat
load Mintime2.mat
load Fuel_Used.mat
load weights_opt2.mat

Minfuel1 = Minfuel1(Minfuel1~=0);
Mintime1 = Mintime1(Minfuel1~=0);
Minfuel1 = Minfuel1(Minfuel1~=Inf);
Mintime1 = Mintime1(Minfuel1~=Inf);
Minfuel1(3) = [];
Mintime1(3) = [];

add = [22752,11.375;34128,11.375;39816,11.375];
add2 = [27342.2, 11]; %5.8 mass 8.0 F 2.5e-4 gamma
add3 = [27263.9, 10.75; 43622.3, 10.75; 38169.5, 10.75]; %5.8 8.2 2.5e-4
input = [Minfuel1,Mintime1;add; add2; add3];
% input = [fuels(1:end-1);errors(1:end-1)];

[membership, member_value]=find_pareto_frontier(input);
scatter(input(:,1),input(:,2),'filled');
hold on
scatter(member_value(:,1),member_value(:,2),'r','filled');
plot(member_value(:,1),member_value(:,2),'r','LineWidth',2);
xlabel('Fuel Usage')
ylabel('Minimum Time')
title('Pareto Frontier Plot for Rocket Landing Optimization')
figure()
%fuels = [fuels(2:end) 4.3282e+04]
loglog(fuels(1:end),pos_errors(1:end),'LineWidth',2)
hold on
%loglog(fuels(1:end),vel_errors(1:end), 'LineWidth',2)
%loglog(fuels(3:end),errors(3:end), 'LineWidth',2)
title('Optimizing Weights for Fuel use and Landing Error')
xlabel('Fuel Usage')
ylabel('Error Magnitude')
legend('Position Errors','Velocity Errors')
% scatter(4.3282e+04,2.2605e-04)
