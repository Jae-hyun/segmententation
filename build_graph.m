function [normals, edges] = build_graph(nodes, g)
    nscans = g.nscans;
    nranges = g.nranges;
    normals = zeros(nscans*nranges,3);
    edges = zeros(nscans*nranges,g.NUM_EDGES);
    for s = 0:1:nscans-1
        for r=0:1:nranges-1
            idx = s* nranges + r;
            %       nodes(idx+1,1) = img_matrix(s+1, r+1);

            if (s == 0 || s == nscans-1 || r == 0 || r == nranges-1 || nodes(idx+1,4) == 0)
                for i=0:1:g.NUM_EDGES-1
                    edges(idx+1, i+1) = g.NO_EDGE;
                end
            else
                edges(idx+1, 5) = (s + 0) * nranges + (r + 0)+1;
                edges(idx+1, 4) = (s + 0) * nranges + (r - 1)+1;
                edges(idx+1, 1) = (s - 1) * nranges + (r - 1)+1;
                edges(idx+1, 2) = (s - 1) * nranges + (r + 0)+1;
                edges(idx+1, 3) = (s - 1) * nranges + (r + 1)+1;
                edges(idx+1, 6) = (s + 0) * nranges + (r + 1)+1;
                edges(idx+1, 7) = (s + 1) * nranges + (r - 1)+1;
                edges(idx+1, 8) = (s + 1) * nranges + (r + 0)+1;
                edges(idx+1, 9) = (s + 1) * nranges + (r + 1)+1;
                % Remove Edges if range == 0
                if nodes(edges(idx+1, 4),4) == 0
                    edges(idx+1, 4) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 1),4) == 0
                    edges(idx+1, 1) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 2),4) == 0
                    edges(idx+1, 2) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 3),4) == 0
                    edges(idx+1, 3) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 6),4) == 0
                    edges(idx+1, 6) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 7),4) == 0
                    edges(idx+1, 7) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 8),4) == 0
                    edges(idx+1, 8) = g.NO_EDGE;
                end
                if nodes(edges(idx+1, 9),4) == 0
                    edges(idx+1, 9) = g.NO_EDGE;
                end
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
                normals(idx+1, 1:3) = compute_normal(s,r, nranges, nodes, edges);
%                 toc
            end
        end
    end
end