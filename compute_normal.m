function n = compute_normal(sidx, ridx,nranges, scans)
    points = zeros(9,3);
    i = 1;
    for k = -1:1:1
        for j=-1:1:1
            idx = (sidx+k) * nranges + (ridx+j);
            points(i,:) = scans(idx+1,1:3);
            i = i + 1;
        end
    end
    [coeff] = princomp(points);
    n = coeff(:,3);
end