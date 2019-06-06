function [minfuel, time] = get_minfuel(Fmax, gamma, m)

%% Constants
h = 0.5/4;
g = 9.8;
% Fmax = 8e6;
p0 = [1.5e3, 1e3, 3e3]';
v0 = [-300, 400, -400]';
% m = 6e4;
alpha = 0.7;
K = 40*4;

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
    
    minimize(fuel_usage);
    subject to
        p(:,1) == p0;
        v(:,1) == v0;
        p(:,K) == [0,0,0]';
        v(:,K) == [0,0,0]';
        for i = 1:K
            p(:,i+1) == p(:,i) + (h/2)*(v(:,i) + v(:,i+1))
            v(:,i+1) == v(:,i) + (h/m)*f(:,i) - h*[0,0,g]';
            v(:,i+1) == v(:,i) + (h/m)*f(:,i) - h*[0,0,g]';
            force_scale*norm(f(:, i),2) <= force_scale*Fmax;
            p(3,i) >= alpha*norm(p(1:3,i),2);            
        end        
cvx_end

minfuel = cvx_optval;
time = K*h;
end

    