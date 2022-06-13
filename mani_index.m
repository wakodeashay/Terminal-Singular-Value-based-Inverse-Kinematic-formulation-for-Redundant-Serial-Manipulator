function mi = mani_index(r,q)
    p = r.jacob0(q) ; 
    t = p(1:2,:);
    last_row = [1,1,1,1];
    jac = [t;last_row];
    mi = sqrt(det(jac*transpose(jac)));
end