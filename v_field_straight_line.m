%code for vector field %straight line
w1=[0;0];    							 %waypoint 1
w2=[1200;1200];     							 %waypoint 2
zi_f=atan2((w2(2)-w1(2)) ,(w2(1)-w1(1)));  			%the angle of line to be followed with x axis
zi_e=pi/4 ;              %entry anglel to the transition region
n=input('enter number of agents: ');
store=zeros(2,n,100001);
store(:,:,1)=input('enter initial coordinates: '); %initial position of agents


%s_star=(dot((z-w1),(w2-w1)))/(norm(w2-w1))^2 ;    	%calculate position of MAV along path

m=(w2(2)-w1(2))/(w2(1)-w1(1));
c=w1(2)-w1(1)*m;
for i=1:n
    e(i)=(store(2,i,1)-m*store(1,i,1)-c)/sqrt(1+m^2);   %norm(z-(s_star*(w2-w1)+w1));      				%calculate distance of MAV from path
end

jd=w2-w1;
for i=1:n
    ram=store(:,i,1)-w1;   
    signn=cross([jd(1) jd(2) 0],[ram(1) ram(2) 0]);     				%Calculate which side of path MAV is on
    rho(i)=sign(signn(3));
end
        
tau=50;             									%transition region boundary distance
k=1.3; 								% greater than 1 % to control the rate of transition
S=10;  	                            %magnitude of air speed
zi=zeros(n);
zi=input('enter initial heading: ');%initial zi 
alpha=3;
delta_t=0.001;

	for t=1:100000
        for j=1:n

		
        if (abs(e(j))>tau)
				zi_c=zi_f-rho(j)*zi_e;
		else
				zi_c=zi_f - zi_e*power((e(j))/tau , k) - power((e(j)), (k-1))*sin(zi(j))*(k*zi_e*S)/(alpha*power(tau,k));
		end		        
		
		zi_dot=alpha*(zi_c-zi(j));
		zi(j)=zi_dot*delta_t+zi(j);
		x_dot=S*cos(zi(j));
		x=x_dot*delta_t +store(1,j,t);
		y_dot=S*sin(zi(j));
		y=y_dot*delta_t+store(2,j,t);
		store(1,j,t+1)=x(1);
		store(2,j,t+1)=y(1);
        
        
         e(j)=(store(2,j,t+1)-m*store(1,j,t+1)-c)/sqrt(1+m^2);   %norm(z-(s_star*(w2-w1)+w1));      				%calculate distance of MAV from path
       
          
        end
    end

    %for plotting
for i=1:n
    for p=1:100001
        pk=store(:,i,p);
        xx(p)=pk(1);
        yy(p)=pk(2);
    end
    plot(xx(1),yy(1),'bs'); %initial position of agents is denoted by squares
    hold on
    plot(xx(100001),yy(100001),'bo') % final position of agents is denoted by circles
    plot(xx,yy)  %trajectory of agents
end
xlim([-100 1200]);
ylim([-100 1200]);
plot([w1(1) w2(1)],[w1(2) w2(2)],'r-');
clear;