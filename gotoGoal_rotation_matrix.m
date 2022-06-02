close all
clear all
t=0;
dt=0.1;
T=100;
xo=[25,20]; % CW/CCW depends on the robot position
xg=[90,90];
x=[0,0];
K=0.1;
figure
drawnow
speedSet=[];
tSet=[];
vo=5;
a=0.1;
c=100;
ep=0.1;
hold on
plot(xo(1),xo(2),'r*')
plot(xg(1),xg(2),'gd')

hold on
xCircleSet=[];
N=100;
n=1;
R=5;
b=1;
delta=10;    % safety distance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% obstacle creation
while n<N
    theta=2*pi/N*n;
    xCircle=xo+R*[cos(theta),sin(theta)];
    plot(xCircle(1),xCircle(2),'r*')
    hold on
    xCircleSet=[xCircleSet;xCircle];
    n=n+1;
end

xo=[60,50];

n=1;
while n<N
    theta=2*pi/N*n;
    xCircle=xo+3*R*[cos(theta),sin(theta)];
    plot(xCircle(1),xCircle(2),'r*')
    hold on
    xCircleSet=[xCircleSet;xCircle];
    n=n+1;
end

xo=[50,70];

n=1;
while n<N
    theta=2*pi/N*n;
    xCircle=xo+1.5*R*[cos(theta),sin(theta)];
    plot(xCircle(1),xCircle(2),'r*')
    hold on
    xCircleSet=[xCircleSet;xCircle];
    n=n+1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

mode=0;%GTG mode
while t<T
    
    sC=size(xCircleSet,1);  % size(xCircleSet,1) --> number of Row, xCircleSet --> obstacle points
    obsDistMin=inf;
    for s=1:sC  % For each obstacle points, find the nearest obstacle point to the robot, set that xo
        xC=xCircleSet(s,:);
        obsDist=norm(xC-x);
        if obsDist<obsDistMin
            obsDistMin=obsDist;
            minS=s;
        end
    end
    xo=xCircleSet(minS,:);
    e=xo-x;
    eG=xg-x;
    Kg=vo*(1-exp(-a*norm(e)^2))/norm(e);
    
    K=1/norm(e)*(c/(norm(e)^2+ep));         % Coefficient that allows the robot's velocity to be increased in the direction of moving away from the obstacle when the distance between the robot and obstacle is getting decreased
    % 회피할때 오히려 속도가 더 커지는 것은 충돌 회피 제어기에서 c 때문임
    % c를 작게하면 충돌 회피하는 속도가 느려짐
    uCC=([0 -1;1 0]*K*e')'; % avoiding-obstacle control X rotation-matrix
    uC=([0 1;-1 0]*K*e')';  % CW control
    uGTG=Kg*eG;
    uAO=-K*e';              % avoiding-obstacle control
    if mode==0
        u=uGTG;
    end
    if mode==1
       u=uCC; 
    end
    if mode==2;
       u=uC; 
    end
    if norm(e)<=delta && mode==0 && dot(uCC,uGTG)>0 %에러가 델타보다 작고 GTG상태였고 GTG방향과 반시계방향으로 도는 제어기가 같은 방향이라고 하면
        u=uCC;  % 그때 모드는 반시계방향으로 도는 제어기(장애물회피)
        mode=1;%CC
        dr=norm(xg-x); % 이때가 모드가 바뀔때, d_tau
    elseif norm(e)<=delta && mode==0 && dot(uC,uGTG)>0
        u=uC;
        mode=2;%C
        dr=norm(xg-x);
    end
    if mode==1 || mode==2 % C or CC, 시계방향으로 돌거나 반시계방향으로 돌고 있을 때
        if norm(xg-x)<dr && dot(uAO,uGTG)>0     % 로봇과 장애물과의 거리가 dr보다 작고 AO의 방향과 GTG의 방향이 같은 방향이면.
            mode=0;
            u=uGTG;
        end
    end
    
    %u=e/norm(e)*(c/norm(e)^2+ep)
    %norm(u)=c/(norm(e)^2+ep)
    %e=0->norm(u)=c/ep;     장애물과의 거리가 클 때 장애물 회피 움직임이 더 크게 작용
    %e=inf->norm(u)=0       장애물과의 거리가 멀어지면 멀어질수록 제어입력의 크기는 영
    
    xDot=u;         % 제어입력 u는 xDot이 되고
    speed=norm(u);  % u의 크기는 speed가 되고
    
    x=x+dt*xDot;    % x를 업데이트
    plot(x(1),x(2),'bo');
    hold on
    
    if norm(xo-x)<1  %20보다 크면 break로 코드가 종료
        break
    end
    t=t+dt;
    tSet=[tSet t];
    speedSet=[speedSet speed];
end
%일정한 속도로 가다가 장애물 회피할 때는 속도를 조금 늦춤
%최대 속도로 쭉 가다가 목표지점에 가까울수록 속도를 늦춤
xlabel('x(m)')
ylabel('y(m)')

figure
plot(tSet, speedSet)
xlabel('t(sec)')
ylabel('speed(m/s)')