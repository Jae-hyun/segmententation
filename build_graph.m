function [normals, edges] = build_graph(nodes, g)
    nscans = g.nscans;
    nranges = g.nranges;
    normals = zeros(nscans*nranges,3);
    edges = zeros((nscans-1)*(nranges-1)*g.NUM_EDGES, 4);
    num = 1;

    for s = 0:1:nscans-1
        for r=0:1:nranges-1
            idx = s* nranges + r;
            if r < nranges -1
                idy = s * nranges + (r+1);
                edges(num,1) = idx + 1;
                edges(num,2) = idy + 1;
                if nodes(idx+1,4) == 0 || nodes(idy+1,4) == 0
                    edges(num,3) = g.NO_EDGE;
                else
                    edges(num,3) = pdist([nodes(edges(num,1),1:3); nodes(edges(num,2),1:3)]);
                end
                num = num + 1;
            end
            if s < nscans -1
                idy = (s+1) * nranges + (r);
                edges(num,1) = idx + 1;
                edges(num,2) = idy + 1;
                if nodes(idx+1,4) == 0 || nodes(idy+1,4) == 0
                    edges(num,3) = g.NO_EDGE;
                else
                    edges(num,3) = pdist([nodes(edges(num,1),1:3); nodes(edges(num,2),1:3)]);
                end
                num = num + 1;
            end
            if s < nscans -1 && r < nranges -1
                idy = (s+1) * nranges + (r+1);
                edges(num,1) = idx + 1;
                edges(num,2) = idy + 1;
                if nodes(idx+1,4) == 0 || nodes(idy+1,4) == 0
                    edges(num,3) = g.NO_EDGE;
                else
                    edges(num,3) = pdist([nodes(edges(num,1),1:3); nodes(edges(num,2),1:3)]);
                end
                num = num + 1;
            end
            if s > 0 && r < nranges -1 && nodes(((s-1) * nranges + (r+1)) + 1,4) ~= 0
                idy = (s-1) * nranges + (r+1);
                edges(num,1) = idx + 1;
                edges(num,2) = idy + 1;
                if nodes(idx+1,4) == 0 || nodes(idy+1,4) == 0
                    edges(num,3) = g.NO_EDGE;
                else
                    edges(num,3) = pdist([nodes(edges(num,1),1:3); nodes(edges(num,2),1:3)]);
                end
                num = num + 1;
            end
        end
    end
    for s = 0:1:nscans-1
        for r=0:1:nranges-1
            idx = s* nranges + r;
            if (s == 0 || s == nscans-1 || r == 0 || r == nranges-1 || nodes(idx+1,4) == 0)
                
            else
            % Comput normals
%                 tic
                normals(idx+1, 1:3) = compute_normal(s,r, nranges, nodes);
%                 toc
            end
        end
    end

end