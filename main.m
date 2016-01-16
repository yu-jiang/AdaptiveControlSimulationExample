function main()
%% This is an example of the simulation of a simple adaptive 
% control system
x0 = 1;         % Initial conidtion for the state
theta0 = [0;0]; % Initial condition for the estimate
a = 2; b = 3;   % Parameters unknown to the controller
K = 1;          % controller parameter
Gamma = diag([1,2]);    % controller parameter
[t,y] = ode45(@(t,x)mySystem(t,x,a,b,K,Gamma), ...
    [0 100], ...  % Simulation time span
    [x0;theta0]);

figure
subplot(311)
plot(t,y(:,1),t,sin(t));
legend({'$x$','$x_d$'},'Interpreter','latex');

subplot(312)
plot(t,y(:,2), [0 t(end)], [a a])
legend({'$\hat{a}$','$a$'},'Interpreter','latex');

subplot(313)
plot(t,y(:,3),  [0 t(end)], [b b])
legend({'$\hat{b}$','$b$'},'Interpreter','latex');
end

% The function describing the syetem dynamics
function dstate = mySystem(t,state,a,b,K,Gamma)
x = state(1);
htheta = state(2:3);
xd = sin(t);
xdd = cos(t);
e = x - xd;

u = -Local_psi(x)*htheta + xdd - K*e;

dhtheta = Gamma*Local_psi(x)'*e;

dx = -a*x*x + b*exp(-x) + u;

dstate = [dx; dhtheta];
end

% Local function
function y = Local_psi(x)
y = [-x*x exp(-x)];
end
