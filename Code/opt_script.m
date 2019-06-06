clear all
close all

Fmax = linspace(8e6,8.5e6,3);% 8e6, 2e-4, 5.75e4, 8.25e6, 3e-4, 6e4
gamma = linspace(2e-4,4e-4,2);
mass = linspace(5.5e4,6.0e4,3); %8e6, 

Minfuel2 = zeros(length(Fmax),length(mass),length(gamma));
Mintime2 = Minfuel2;
time = Minfuel2;
Fuel_Used = Mintime2;

for i = 1:length(Fmax)
    for j = 1:length(mass)
        for k = 1:length(gamma)
%              A = get_minfuel(Fmax(i), gamma(k), mass(j))
%              Minfuel2(i,j,k) = A(1);
%              time(i,j,k) = A(2);
             fprintf('iter done')
             [A, B] = get_mintime(Fmax(i),gamma(k), mass(j));
             Mintime2(i,j,k) = A;
             Fuel_Used(i,j,k) = B;
        end
    end
end

% Minfuel2 = reshape(Minfuel2,length(Fmax)*length(gamma)*length(mass),1);
Fuel_Used = reshape(Fuel_Used,length(Fmax)*length(gamma)*length(mass),1);
Mintime2 = reshape(Mintime2,length(Fmax)*length(gamma)*length(mass),1);

% save('Minfuel2.mat','Minfuel2')
save('Mintime2.mat','Mintime2')
save('Fuel_Used.mat','Fuel_Used')
