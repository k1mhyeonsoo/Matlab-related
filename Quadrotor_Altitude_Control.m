close all
clear all

c = 1;      %
g = 9.8;
rho = 1.275;
A = 1;
CL = 0.4;

m = 10;   % drone mass
x = 0;      % drone height
xDot = 0;   % drone velocity
dt = .1;     % sampling interval
t = 0;
T = 1000;
r = 10;     % reference height
u_max = m*g*1;
xSet = [];
tSet = [];
uSet = [];

bb = 0;
eIntegral = 0;
%% PID parameters
k = 1;
ki = .1;
kd = 10;
%%
propelVSet = [];
while t < T
    e = r - x;
    if t==0
        prev_e = e;
    end
    eIntegral = eIntegral + dt*e;
    eDot = (e - prev_e)/dt;
    %% control input
    if bb == 1      % bangbang control
        if e>0
            u = u_max;
        elseif e<0
            u = -u_max;
        else
            u = 0;
        end
    else            % PID regulator
        u = k*e + ki*eIntegral + kd*eDot + m*g;
    end
    %%
%     xDDot = c*u - g;
    xDDot = (u - m*g)/m;
    xDot = xDot + dt*xDDot;
    x = x + dt*xDot;
    t = t + dt;
    xSet = [xSet x];
    tSet = [tSet t];
    uSet = [uSet u];
    % u = 0.5*CL*v^2*A*rho
    propelV = sqrt(u*2/A/CL/rho);
    propelVSet = [propelVSet propelV];
    
    prev_e = e;
end

figure(1)
subplot(211)
plot(tSet, xSet,'Linewidth',2.5);
xlabel('time(sec)');
ylabel('state(m/s)');
grid on

subplot(212)
plot(tSet, uSet,'Linewidth',2.5);
xlabel('time(sec)');
ylabel('input(N)');
grid on

figure(2)
plot(tSet,propelVSet,'Linewidth',2.5);
xlabel('time(sec)');
ylabel('propelV');
grid on
