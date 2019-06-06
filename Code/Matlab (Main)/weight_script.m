weight_fuel = [linspace(1,20,80)];
weight_error = 1;
fuels = zeros(size(weight_fuel));
pos_errors = fuels;
errors = fuels;
vel_errors = fuels;
for i = 1:length(weight_fuel)
             [A, B, C] = minimize_error_fuel2(weight_fuel(i), weight_error);
             fuels(i) = A;
             pos_errors(i) = B;
             vel_errors(i) = C;
             errors(i) = B + C;
            i
end

%% save
% save('Minfuel2.mat','Minfuel2')
save('weights_opt2.mat','fuels', 'errors', 'weight_fuel','pos_errors','vel_errors')

