function mi = tsv(r,q)
    J=r.jacob0(q);
    [U,S,V] = svd(J);
    mi = S(3,3);
end