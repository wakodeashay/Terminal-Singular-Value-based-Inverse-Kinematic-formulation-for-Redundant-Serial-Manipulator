function l = l_bound(q)
    qmin=[-2*pi/3,-2*pi/3,-2*pi/3,-2*pi/3];
    q_dotmin = [-2,-2,-2,-2];
    rho_s = 5;
    rho_i = 2.5;
    n=length(q);
    l=zeros(n,1);
    for i=1:n
        if q(i)-qmin(i) <= rho_i
            l(i) = q_dotmin(i)*(q(i)-qmin(i)-rho_s)/(rho_i-rho_s);
        else
            l(i) = q_dotmin(i);
        end   
    end
end