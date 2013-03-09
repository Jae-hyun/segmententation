function [normals, edges] = build_graph(nodes, labels, g)
    nscans = g.nscans;
    nranges = g.nranges;
    normals = zeros(nscans*nranges,3);
    edges = zeros((nscans-1)*(nranges-1)*g.NUM_EDGES, 4);
    num = 1;

    for s = 0:1:nscans-1
        for r=0:1:nranges-1
            idx = s* nranges + r;
            % Edge Assign
            if labels(idx+1) ~= g.label.gound
                if r < nranges -1
                    idy = s * nranges + (r+1);
                    if labels(idy+1) ~= g.label.gound
                        edges(num,1) = idx + 1;
                        edges(num,2) = idy + 1;
                        num = num + 1;
                    end
                end
                if s < nscans -1
                    idy = (s+1) * nranges + (r);
                    if labels(idy+1) ~= g.label.gound
                        edges(num,1) = idx + 1;
                        edges(num,2) = idy + 1;
                        num = num + 1;
                    end
                end
                if s < nscans -1 && r < nranges -1
                    idy = (s+1) * nranges + (r+1);
                    if labels(idy+1) ~= g.label.gound
                        edges(num,1) = idx + 1;
                        edges(num,2) = idy + 1;
                        num = num + 1;
                    end
                end
                if s > 0 && r < nranges -1
                    if labels(idy+1) ~= g.label.gound
                        idy = (s-1) * nranges + (r+1);
                        edges(num,1) = idx + 1;
                        edges(num,2) = idy + 1;
                        num = num + 1;
                    end
                end
            end
        end
    end
    %% Surface Normals calculation using PCA
    %{
    
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
    %}

end