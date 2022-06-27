%% term project Kalaman Filter Design
%% B688018 electrical engineering kim hyeon-soo
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3-dimension position estimation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
t=0;
dt=.1;
T=25;

truePosition=[5000
              5000
              5000
              5.0
              6.6
              6.7 ];   % real position(x;y;z;xDot;yDot;zDot)   % 6x1 matrix
          
A=[1 0 0 dt 0 0
   0 1 0 0 dt 0
   0 0 1 0 0 dt
   0 0 0 1 0 0
   0 0 0 0 1 0
   0 0 0 0 0 1]; % 6x6 matrix

B=[(1/2*dt^2) 0 0
    0 (1/2*dt^2) 0
    0 0 (1/2*dt^2)
    dt 0 0
    0 dt 0
    0 0 dt];         % 6x3 matrix

H=[1 0 0 0 0 0
   0 1 0 0 0 0
   0 0 1 0 0 0];       % estimate positions only

estimatedPosition=[5000.1; 5000.2; 5000.3; 5.1; 6.5; 5.7];  % initial estimation          % 6x1 matrix
figure
plot3(estimatedPosition(1),estimatedPosition(2),estimatedPosition(3),'ro','LineWidth',10)
hold on

processNoise=10;
noise=7.5;

estimatedErrorCo=diag([1000^2,1000^2,1000^2,10^2,10^2,10^2]);       % estimatedErrorCo = P(6x6)
measuredErrorCo=diag([noise^2,noise^2,noise^2]);                    % measuredErrorCo = R(3x3)
ErrorCo=diag([processNoise^2,processNoise^2,processNoise^2]);       % 3x3 matrix

tSet=[];

while t<T
   randomAccelX=processNoise*(randn);
   randomAccelY=processNoise*(randn);
   randomAccelZ=processNoise*(randn);
   randomAccel=[randomAccelX;randomAccelY;randomAccelZ];
   truePosition=A*truePosition+B*3*randomAccel;   % true position + random accel
   
   t=t+dt;
   
   measuredPosition=H*truePosition+[randn,randn,randn]'*noise;
  % there are always noises in estimation values
  
   %% Kalman Processing
   estimatedPosition=A*estimatedPosition;
   estimatedErrorCo=A*estimatedErrorCo*A'+B*ErrorCo*B';       % P=A*P*A'+B*Q*B';
   kalmanGain=estimatedErrorCo*H'*inv(H*estimatedErrorCo*H'+measuredErrorCo);
   estimatedPosition=estimatedPosition+kalmanGain*(measuredPosition-H*estimatedPosition);
   estimatedErrorCo=(eye(6)-kalmanGain*H)*estimatedErrorCo;
   
   plot3(truePosition(1),truePosition(2),truePosition(3),'h','linewidth',1);
   hold on
   plot3(estimatedPosition(1),estimatedPosition(2),estimatedPosition(3),'p','linewidth',1);
   hold on
   plot3(measuredPosition(1),measuredPosition(2),measuredPosition(3),'*');
   hold on
   
   
   
   axis equal
   hold on
   
   tSet=[tSet t];
end

% insertColor={'#FF0','#0B0','#00F'}
insertColor=[rand rand rand;rand rand rand;rand rand rand]
colororder(insertColor)
legend('First Estimated Position','True Position','Estimated Position','Measured Position');

title('Position Estimation(3-dimension)')
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3-dimension position estimation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3-dimension velocity estimation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close all
% clear all
% t=0;
% dt=.1;
% T=50;
% trueVelocity=[4123;6756;3450; 200.0; 200.0; 200.0 ];   % real velocity(x;y;z;xDot;yDot;zDot)   % 6x1 matrix
% A=[1 0 0 dt 0 0; 0 1 0 0 dt 0; 0 0 1 0 0 dt;0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]; % 6x6 matrix
% B=[(0.5*dt^2) 0 0; 0 (0.5*dt^2) 0; 0 0 (0.5*dt^2); dt 0 0; 0 dt 0; 0 0 dt];         % 6x3 matrix
% % H=[1 0 0 0 0 0;0 1 0 0 0 0; 0 0 1 0 0 0];
% H=[0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]; 
% 
% estimatedVelocity=[0; 0; 0; 190.0; 190.0; 190.0];  % initial estimation                   6x1 matrix
% 
% figure
% plot3(estimatedVelocity(4),estimatedVelocity(5),estimatedVelocity(6),'rs','LineWidth',10);
% hold on
% 
% processNoise=5;
% noise=1;
% 
% estimatedErrorCo=diag([0,0,0,100^2,100^2,100^2]);    % estimatedErrorCo = P(6x6)
% measuredErrorCo=diag([noise^2,noise^2,noise^2]);              % measuredErrorCo = R(3x3)
% ErrorCo=diag([processNoise^2,processNoise^2,processNoise^2]); % 3x3 matrix
% 
% tSet=[];
% 
% while t<T
% %    randomAccelX=processNoise*randn;
% %    randomAccelY=processNoise*randn;
% %    randomAccelZ=processNoise*randn;
%    
% randomAccelX=processNoise*abs(randn);
% randomAccelY=processNoise*abs(randn);
% randomAccelZ=processNoise*abs(randn);
% randomAccel=[randomAccelX;randomAccelY;randomAccelZ];
% trueVelocity=A*trueVelocity+B*randomAccel;
% 
% 
% %    trueVelocity=A*trueVelocity+B*4*randomAccel;   % true velocity
% %    trueVelocity=A*trueVelocity; %+B*randomAccel;   % true velocity
% t=t+dt;
% 
%    if trueVelocity(4)>230 && trueVelocity(5)>230 && trueVelocity(6)>230
%       trueVelocity(4)=230+5*abs(randn);
%       trueVelocity(5)=230+5*abs(randn);
%       trueVelocity(6)=230+5*abs(randn);
%    end
%    
%    
%    measuredVelocity=H*trueVelocity+[randn,randn,randn]*noise;
%    
%    %% Kalman Filter Processing
%    estimatedVelocity=A*estimatedVelocity;
%    estimatedErrorCo=A*estimatedErrorCo*A'+B*ErrorCo*B';       % P=A*P*A'+B*Q*B';
%    kalmanGain=estimatedErrorCo*H'*inv(H*estimatedErrorCo*H'+measuredErrorCo);
%    estimatedVelocity=estimatedVelocity+kalmanGain*(measuredVelocity-H*estimatedVelocity);
%    estimatedErrorCo=(eye(6)-kalmanGain*H)*estimatedErrorCo;
%    
%    plot3(trueVelocity(4),trueVelocity(5),trueVelocity(6),'p','linewidth',1.5);
%    hold on
%    plot3(estimatedVelocity(4),estimatedVelocity(5),estimatedVelocity(6),'h','linewidth',1.5);
%    hold on
%    plot3(measuredVelocity(1),measuredVelocity(2),measuredVelocity(3),'*','linewidth',1.0);
%    hold on
% 
% % plot(t,trueVelocity(4),'o','linewidth',1.5)
% % hold on
% % plot(t,trueVelocity(5),'d','linewidth',1.5)
% % hold on
% % plot(t,trueVelocity(6),'s','linewidth',1.5)
% % hold on
% 
%    axis equal
%    hold on
%    
%    tSet=[tSet t];
% end
% 
% insertColor={'#0f8','#00f','#F1C'}
% % insertColor=[rand rand rand;rand rand rand;rand rand rand]
% colororder(insertColor)
% 
% % legend('True Velocity(X)','True Velocity(Y)','True Velocity(Z)')
% % title('True Velocity(each elements)')
% % xlabel('Time(sec)')
% % ylabel('Velocity(m/s)')
% 
% legend('First Estimated Velocity','True Velocity','Estimated Velocity','Measured Velocity');
% title('Velocity Estimation(3-dimension)');
% xlabel('X-axis')
% ylabel('Y-axis')
% zlabel('Z-axis')
% grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3-dimension velocity esetimation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
