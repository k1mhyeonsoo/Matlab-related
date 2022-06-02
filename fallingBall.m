%%
close all
clear all

t=0;
dt=0.01;
h=10;
g=9.8;
gamma=0.8;
A=[0 1;0 0];
B=[0;-g];
R=[1 0;0 -gamma];
x=[h;0];
ySet=[];
tSet=[];
while t<10
    xDot=A*x+B;
    x=x+xDot*dt;
    y=[1 0]*x;
    
    if y<=0 && x(2)<=0
        x=R*x;
    end
    
    ySet=[ySet y];
    tSet=[tSet t];
    t=t+dt;
end
figure
plot(tSet,ySet,'r*');
legend('h');
xlabel('t(sec)');
ylabel('h(meter)');
grid on