function [m_s,q_s,qdot_s,l_s,u_s]=original_formulation(alpha)
    %% Defining Manipulator - 2-Dimensional 3R manipulator
    Ls1 = Revolute('d', 0, 'a', 100, 'alpha', 0);
    Ls2 = Revolute('d', 0, 'a', 75, 'alpha', 0);
    Ls3 = Revolute('d', 0, 'a', 50, 'alpha', 0);
    Ls4 = Revolute('d', 0, 'a', 25, 'alpha', 0);
    s = SerialLink([Ls1 Ls2 Ls3 Ls4]);
    n=4;
    
    %% Defining Parameters 
    %alpha=50;
    delta_q= 0.001 ;
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
    y= [pi/12,pi/3,pi/3,0];
    q(1,:)=y;
    qdot=zeros(p,n);
    % Manipulability index
    Manipulability_Index = zeros(p,1);
    Manipulability_Index(1)=new_mani_index(s,q(1,:));
    % Defining Matrices for Optimization Problem
    Q = eye(n);
    A = [ ];
    b = [ ];
    l=zeros(p,1);
    u=zeros(p,1);
    for j=1:p
        f = -alpha*T_samp*mani_grad_s(s,q(j,:),delta_q ,n);
        Aeq = s.jacob0(q(j,:));
        %beq = [4*Dx/(T_total^(2))*(-j^(3)/(T_total^(2)*10^(6))+j/100),4*Dy/(T_total^(2))*(-j^(3)/(T_total^(2)*10^(6))+j/100),4*theta/(T_total^(2))*(-j^(3)/(T_total^(2)*10^(6))+j/100)];
        %beq= [35*cos(2*pi*j/600),35*sin(2*pi*j/600),0];
        Manipulability_Index(j) = new_mani_index(s,q(j,:));
        l=lowerb_s(q(j,:));
        u=upperb_s(q(j,:));
        [x] = quadprog(Q,f,A,b,[Aeq(1:2,:);1,1,1,1],beq,l,u);
        qdot(j,:) = x ;
        q(j+1,:) = q(j,:) + T_samp*transpose(x) ;   
    end 
    m_s = Manipulability_Index;
    q_s = q;
    qdot_s = qdot;
    l_s=l;
    u_s=u;
end