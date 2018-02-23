

% {obstacle avoidance}%



n=input('number of agents: ');

initial=zeros(1,2);      % {coordinates of target}%
initial(1,1:2)=input('enter target position: ');

x=zeros(1,n);
x(1,1:n)=input('Enter initial x coordinates :');      % {an array that stores x coordinates of n agents at every iteration. And at end of every iteration this array is copied to the xx matrix}%


y=zeros(1,n);
y(1,1:n)=input('Enter initial y coordinates :');      %{an array that stores x coordinates of n agents at every iteration. And at end of every iteration this array is copied to the yy matrix}%


theta=zeros(1,n);
theta(1,1:n)=input('Enter initial heading angle:');       % {this array stores initial heading angle that is angle which the velocity vector makes with reference line.}%


v=zeros(1,n);
v(1,1:n)=input('Enter velocity:');           % {this array stores velocity of agents}%

xx=zeros(200000,n);       % {matrix in which Each ith row stores x coordinate of n agents.}%
xx(1,1:n)=x(1,1:n); % {copying initial x coordinates to xx matrix}%

yy=zeros(200000,n);       % {matrix in which Each ith row stores y coordinate of n agents.}%
yy(1,1:n)=y(1,1:n);



dfromt=zeros(200000,n);                                    % {this matrix stores distance of agent from target}%
dfromt(1,1:n)=((x(1,1:n)-initial(1)).^2+(y(1,1:n)-initial(2)).^2).^.5;      % {initial distance of agents from target}%

m=input('Enter number of obstacle  :');
obstacle=zeros(m,2);
for i=1:m
obstacle(i,1:2)=input('Enter coordinates of obstacle :');
end

k=zeros(1,m);
k(1,1:m)=input('Enter constant of potential function for each obstacle: ');

radius=zeros(1,m);
radius(1,1:m)=input('Enter radius of influence of each obstacle:  ');

dfromo=zeros(200000,n,m);        %{ distance of agents from obstacles}%
for i=1:m
dfromo(1,1:n,i)=((x(1,1:n)-obstacle(i,1)).^2+(y(1,1:n)-obstacle(i,2)).^2).^.5;      % {initial distance of agents from obstacles}%
end


k1=.02;            % {k1 is control gain .For a particular initial condition ,for some range of values of k a circle will be formed around the target,but for other values of k a diverging solution is obtained}%

d=.01;             %  {time interval of  simulation .}%

for i=2:200000      %{iterating 200000 times }%

for j=1:n          %{ in each iteration of above for   loop  ,this for loop runs n times to find phi_it,phi_ii, w,x ,y ,theta for each agent}%

% {finding phi_it of ith agent which  is bearing angle between ith agent and target,that is it the  angle between, line joining ith agent and target, and velocity vector of ith agent}%

phi_it=atan2((initial(2)-y(j)),(initial(1)-x(j)))-theta(j);    %{first term is alpha which is angle between, line joining i and target ,and reference line and second term is heading angle which is angle between velocity vector and reference line}%




% {finding phi_ii which is bearing angle   between i and i+1  that is it the  angle between, line joining ith agent and i+1 th , and velocity vector of ith agent}%

phi_ii=atan2(( y(mod(j,n)+1)-y(j)),(x(mod(j,n)+1)-x(j)))-theta(j);     %{first term is alpha which is angle between, line joining i and i+1 ,and reference line and second term is heading angle which is angle between velocity vector and reference line}%



%{ as it is assumed that phi_ii and phi_it are always between 0 and 2*pi, as both theta and alpha are between 0 and 2*pi, so phi_it and phi_ii obtained above are between -2*pi and 2*pi}%

if phi_it<0
phi_it=phi_it+2*pi*ceil((-phi_it)/(2*pi));
end


if phi_ii<0
phi_ii=phi_ii+2*pi*ceil((-phi_ii)/(2*pi));
end


phi=(phi_it+phi_ii)/2;      %{finding average of phi_it and phi__ii and this phi is used in control law}%
% to make phi  between -pi and pi       %{as the above calculated phi is always between 0 and 2*pi }%

urepel=0;
for z=1:m             %{adding potential function to phi if distance between agent and obstacle is less than radius of influence}% 
dobl=sqrt((x(j)-obstacle(z,1))^2+(y(j)-obstacle(z,2))^2);
if dobl<radius(z)
urepel=urepel+k(z)*(1/dobl-1/radius(z))^2;
end
end
phi=phi+urepel;
while phi>pi

phi=phi-2*pi;

end

w=k1*(phi);                % {control law ,where w is angular velocity of agent}%

x(j)=x(j)+v(j)*cos(theta(j))*d;   %{finding x coordinate of ith agent }%

y(j)=y(j)+v(j)*sin(theta(j))*d;   %{finding y coordinate of ith agent}%

theta(j)=theta(j)+w*d;            %{finding new heading angle of ith agent}%

if theta(j)>2*pi                 %{ensuring theta is between 0 and 2*pi}%

theta(j)=theta(j)-2*pi*floor(theta (j)/(2*pi));

end

if theta(j)<0

theta(j)=theta(j)+2*pi*ceil((-theta (j))/(2*pi));

end

end

for h=1:n

xx(i,h)=x(h);     %{copying array x to ith row of xx matrix}%

yy(i,h)=y(h);      %{copying array x to ith row of xx matrix}%

end

dfromt(i,1:n)=((x(1,1:n)-initial(1)).^2+(y(1,1:n)-initial(2)).^2).^.5;    %{ distance of ith agent from target at time t }%
for z=1:m
dfromo(i,1:n,z)=((x(1,1:n)-obstacle(z,1)).^2+(y(1,1:n)-obstacle(z,2)).^2).^.5;      % { distance of agents from obstacles}%
end
end


figure(1)                           %{plotting trajectory of all agents}%
plot(xx(1,1:n),yy(1,1:n),'k*')
hold on
for u=1:n
plot(xx(1:200000,u),yy(1:200000,u))
hold on
end
plot(xx(200000,1:n),yy(200000,1:n),'ko')
hold off

figure(2)                           %{plotting distance of agents from target}%
for u=1:n
plot((1:200000)*.01,dfromt(1:200000,u))
hold on
end

for u=1:m
figure(u+2)
for y=1:n
plot((1:200000)*.01,dfromo(1:200000,y,u))
hold on
end
hold off
end