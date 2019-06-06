clear all
close all

%% Constants
h = 0.5;
g = 9.8;
m = 5.8e4;
Fmax = 8.2e6;
p0 = [1.5e3, 1e3, 3e3]';
v0 = [-300, 400, -400]';
alpha = 0.7;
gamma = 4e-4;
K = 40;
K_max = K;

%scale force constraint, reduce dynamic range for ECOS solver
force_scale = 1e-7;

for K = 1:K_max + 1
cvx_begin quiet
    variable f(3,K);
    variable p(3,K+1);
    variable v(3,K+1);
    expression fuel_usage;
    fuel_usage = 0;
    for i=1:K
        fuel_usage = gamma*sum(norm(f(:,i),2)) + fuel_usage;
    end
    
    minimize(0);
    subject to
        p(:,1) == p0;
        v(:,1) == v0;
        p(:,K) == [0,0,0]';
        v(:,K) == [0,0,0]';
        for i = 1:K
            p(:,i+1) == p(:,i) + (h/2)*(v(:,i) + v(:,i+1));
            v(:,i+1) == v(:,i) + (h/m)*f(:,i) - h*[0,0,g]';
            force_scale*norm(f(:, i),2) <= force_scale*Fmax;
            p(3,i) >= alpha*norm(p(1:3,i),2);            
        end        
cvx_end
if cvx_status(1:4) == 'Solv'
    break
end
end

disp = ['Time to land is: ', num2str(K*h) , ' sec'];
display(disp)
display(fuel_usage)
%% plotting
x = linspace(-min(p0), max(p0), 50);
y = linspace(-min(p0), max(p0), 50);
px = p(1,:);
py = p(2,:);
pz = p(3,:);
pxp = px(2:3:end);
pyp = py(2:3:end);
pzp = pz(2:3:end);

[X,Y] = meshgrid(x,y);
Z = alpha*sqrt(X.^2 + Y.^2);
surf(X,Y,Z,'FaceAlpha',0.5)
hold on
plot3(px,py,pz,'r','LineWidth',2)
plot3(pxp,pyp,pzp,'bo')
xlabel('x pos (m)')
ylabel('y pos (m)')
zlabel('z pos (m)')
hold on
plot3(px,py,pz,'r','LineWidth',2)
plot3(pxp,pyp,pzp,'bo')
title('Minimum Time Trajectory')

    