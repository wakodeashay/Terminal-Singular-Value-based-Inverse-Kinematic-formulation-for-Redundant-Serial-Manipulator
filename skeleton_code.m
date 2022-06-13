%% Defining Manipulator - 2-Dimensional 4R manipulator
Ls1 = Revolute('d', 0, 'a', 100, 'alpha', 0);
Ls2 = Revolute('d', 0, 'a', 75, 'alpha', 0);
Ls3 = Revolute('d', 0, 'a', 50, 'alpha', 0);
Ls4 = Revolute('d', 0, 'a', 25, 'alpha', 0);
s = SerialLink([Ls1 Ls2 Ls3 Ls4]);
n=4;
%% Defining Parameters 
alpha=25;
delta_q= 0.01 ;
% Distance to be travellled in x and y direction (mm) ; theta is angular
% rotation to be achieved for end-effector (rad)
Dx=-150;
Dy=-300;
theta= pi/2;
% Sampling Time
T_samp = 0.01;
% Total Time
T_total = 6;
% No of iterations
p=T_total/T_samp;

%% Defining Problem
% Joint-Angle Trajectory
q=zeros(p+1,n);
% Initial pose
y= [pi/12,pi/6,pi/6,0];
q(1,:)=y;
qdot=zeros(p,n);
qdot0=[0,0,0,0];
% Manipulability index
Manipulability_Index = zeros(p,1);
Manipulability_Index(1)=mani_index_s(s,q(1,:));
% Defining Matrices for Optimization Problem
Q = eye(n);
for j=1:p
    disp(j)
    f = -alpha*T_samp*mani_grad_s_new(s,q(j,:),delta_q ,n);
    Aeq = s.jacob0(q(j,:));
    % End-Effector Velocity for translational trajectories
    %beq = [4*Dx/(T_total^(2))*(-j^(3)/(T_total^(2)*10^(6))+j/100),4*Dy/(T_total^(2))*(-j^(3)/(T_total^(2)*10^(6))+j/100),4*theta/(T_total^(2))*(-j^(3)/(T_total^(2)*10^(6))+j/100)];
    % End-effector Velocity for circular trajectories
    %beq=[35*cos(2*pi*j/600),35*sin(2*pi*j/600),0];
    Manipulability_Index(j) = mani_index_s(s,q(j,:)) ;
    [x] = quadprog(Q,f,[],[],[Aeq(1:2,:);1,1,1,1],beq,lowerb_s(q(j,:)),upperb_s(q(j,:)) ) ;
    qdot(j,:) = x ;
    q(j+1,:) = q(j,:) + T_samp*transpose(x) ;  
end 