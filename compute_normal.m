function n = compute_normal(sidx, ridx,nranges, scans)
    points = zeros(9,3);
    i = 1;
    for k = -1:1:1
        for j=-1:1:1
            idx = (sidx+k) * nranges + (ridx+j);
%             id = sidx* nranges + ridx;
%             if i < 6
%                 if edges(id+1,i) == idx+1
%             pos = [scans(idx+1,1:3);scans(id+1, 1:3)];
%              dis_p2p = pdist(pos ,'euclidean');
%              if dis_p2p < 3.0
%                 if mod(i,2) == 0 || i == 5
                    points(i,:) = scans(idx+1,1:3);
%                 end
%                 end
%             end
            i = i + 1;
        end
    end
    [coeff] = princomp(points);
    n = coeff(:,3);
end