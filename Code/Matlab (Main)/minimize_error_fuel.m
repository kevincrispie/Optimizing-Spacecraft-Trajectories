function [fuel_used, landE, velE] = minimize_error_fuel(weight_fuel, weight_error)

%% Constants
h = 0.5;
g = 9.8;
m = 6e4; %5.4 - 6.1
Fmax = 8e6;%go to 8.6
p0 = [1.5e3, 1e3, 3e3]';
v0 = [-300, 400, -400]';
alpha = 0.3;
gamma = 4e-4; %could do somthing like 0.8 to 4.4, may less on the 4.4 side
K = 40;
b = 1;

%scale force constraint, reduce dynamic range for ECOS solver
force_scale = 1e-7;

cvx_begin quiet
    variable f(3,K);
    variable p(3,K+1);
    variable v(3,K+1);
    expression fuel_usage;
    fuel_usage = 0;
    for i=1:K
        fuel_usage = gamma*sum(norm(f(:,i),2)) + fuel_usage;
    end
    
    land_error = norm([10*p(1,K) 10*p(2,K), 100*p(3,K)],2);
    velocity_error = norm([10*v(1,K) 10*v(2,K), 100*v(3,K)],2);
    minimize(weight_error*land_error + 10*velocity_error + weight_fuel*fuel_usage);
    subject to
        p(:,1) == p0;
        v(:,1) == v0;
%         p(3,K) == 0;
%         v(:,K) == [0,0,0]';
%         velocity_error <= 10;
        for i = 1:K
            p(:,i+1) == p(:,i) + (h/2)*(v(:,i) + v(:,i+1))
            v(:,i+1) == v(:,i) + (h/m)*f(:,i) - h*[0,0,g]';
            force_scale*norm(f(:, i),2) <= force_scale*Fmax;
            p(3,i) >= alpha*norm(p(1:3,i),2);            
        end        
cvx_end
fuel_used = fuel_usage;
landE = land_error;
velE = velocity_error;

end
    