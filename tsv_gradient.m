function m = tsv_gradient(r,q,delta_q ,n)
    m = zeros(n,1) ;
    for i = 1:n
        a=tsv(r,q+delta_q*([zeros(1,i-1),1,zeros(1,n-i)])) ;
        b=tsv(r,q-delta_q*([zeros(1,i-1),1,zeros(1,n-i)])) ;
        m(i)=(a-b)/(2*delta_q) ;
    end
end
