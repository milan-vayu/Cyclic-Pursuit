

% {implicit leader cyclic pursuit with input as number of agents ,initial coordinates of agents, rho, coordinates of target and initial heading angle of agents , assuming all agents travel with same and constant speed}%
% {in this every i agent follows i+1 agent}%

n=input('number of agents: ');

rho=input('enter rho:');     %{to find position of virtual agent}%

initial=zeros(1,2);
initial(1,1:2)=input('enter target position: ');   % {coordinates of target}%




x=zeros(1,n);
x(1,1:n)=input('Enter initial x coordinates :');     % {an array that stores x coordinates of n agents at every iteration. And at end of every iteration this array is copied to the xx matrix}%


y=zeros(1,n);
y(1,1:n)=input('Enter initial y coordinates :');       %{an array that stores x coordinates of n agents at every iteration. And at end of every iteration this array is copied to the yy matrix}%


theta=zeros(1,n);
theta(1,1:n)=input('Enter initial heading angle:');    % {this array stores initial heading angle that is angle which the velocity vector makes with reference line.}%



v=zeros(1,n);
v(1,1:n)=input('Enter velocity:');

dfromt=zeros(100000,n);                   % {this matrix stores distance of agent from target}%
dfromt(1,1:n)=((x(1,1:n)-initial(1)).^2+(y(1,1:n)-initial(2)).^2).^.5;       % {initial distance of agents from target}%

xx=zeros(100000,n);                        % {matrix in which Each ith row stores x coordinate of n agents.}%
xx(1,1:n)=x(1,1:n);                          % {copying initial x coordinates to xx matrix}%


yy=zeros(100000,n);                       % {matrix in which Each ith row stores y coordinate of n agents.}%
yy(1,1:n)=y(1,1:n);



k1=.05;                                     % {k1 is control gain .For a particular initial condition ,for some range of values of k a circle will be formed around the target,but for other values of k a diverging solution is obtained}%
d=.01;                          %  {time interval of  simulation .}%


for i=2:100000                  %{iterating 100000 times }%
        for j=1:n
             xvir=rho*x(mod(j,n)+1)+(1-rho)*initial(1);          % {x position of virtual agent}%
              yvir=rho*y(mod(j,n)+1)+(1-rho)*initial(2);          % {y position of virtual agent}%

    
              phi=atan2((yvir-y(j)),(xvir-x(j)))-theta(j);   % angle between ,line joining i and i+1 agent ,and velocity vector of i agent}%

              % to make phi  between -pi and pi       %{as the above calculated phi is always between -2*pi and 2*pi }%
             if phi>pi
             phi=phi-2*pi;

            end
           if phi<-pi
               phi=phi+2*pi;
           end
             w=k1*(phi);                             % {control law ,where w is angular velocity of agent}%
             x(j)=x(j)+v(j)*cos(theta(j))*d;           %{finding x coordinate of ith agent }%

            y(j)=y(j)+v(j)*sin(theta(j))*d;             %{finding y coordinate of ith agent}%
              theta(j)=theta(j)+w*d;                    %{finding new heading angle of ith agent}%
            if theta(j)>2*pi                 %{ensuring theta is between 0 and 2*pi}%

                  theta(j)=theta(j)-2*pi*floor(theta (j)/(2*pi));

            end

            if theta(j)<0

                  theta(j)=theta(j)+2*pi*ceil((-theta (j))/(2*pi));

            end
       end
       for h=1:n
           xx(i,h)=x(h);                   %{copying array x to ith row of xx matrix}%
           yy(i,h)=y(h);                      %{copying array x to ith row of xx matrix}%
      end
      dfromt(i,1:n)=((x(1,1:n)-initial(1)).^2+(y(1,1:n)-initial(2)).^2).^.5;       %{finding diastance of ith agent from target at time t }%

end


figure(1)                            % plot of trajectory of all agents
plot(xx(1,1:n),yy(1,1:n),'k*')
hold on
for u=1:n
    plot(xx(1:100000,u),yy(1:100000,u))
    hold on
end
plot(xx(100000,1:n),yy(100000,1:n),'ko')

figure(2)                           % plot of distance of agents from target with time
for u=1:n
    plot((1:100000)*.01,dfromt(1:100000,u))
    hold on
end
