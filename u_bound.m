function u = u_bound(q)
    qmax=[2*pi/3,2*pi/3,2*pi/3,2*pi/3];
    q_dotmax = [2,2,2,2];
    rho_s = 5;
    rho_i = 2.5;
    n=length(q);
    u=zeros(n,1);
    for i=1:n
        if qmax(i)-q(i) <= rho_i
            u(i) = q_dotmax(i)*(qmax(i)-q(i)-rho_s)/(rho_i-rho_s);
        else
            u(i) = q_dotmax(i);
        end   
    end
end