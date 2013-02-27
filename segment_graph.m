function [nodes_out, g] = segment_graph(nodes_in, edges, g)
    tic
    
    sort_edges = sortrows(edges, g.method);
    nodes_out = zeros(g.nnodes,1);
    threshold = zeros(g.nnodes, 3);
    idd = 1:1:g.nnodes;
%     maxr = max(nodes(idd,4));
%     minr = min(nodes(idd,4));
%     threshold(idd,1) = g.c*((nodes(idd,4))/4);
    threshold(idd,1) = g.c;
    % threshold(idd,2) = (g.n)./(nodes(idd,4)/10);
    threshold(idd,2) = (g.n);
    threshold(idd,3) = (g.nnc);
    u = universe;
    u.initialize(g.nnodes);
    for i=1:1:g.nedges
        a = u.find(sort_edges(i,1));
        b = u.find(sort_edges(i,2));
        if(sort_edges(i,4) ~= g.NO_EDGE)
            if (a ~= b)
                if g.method == 3
                    if(sort_edges(i,3) < threshold(a,1) && sort_edges(i,3) < threshold(b,1))
                        u.join(a,b);
                        a = u.find(a);
                        threshold(a,1) = sort_edges(i,3) + ( g.c/u.size(a));
                    end
                elseif g.method == 4
%                 if(sort_edges(i,4) < threshold(a,2) && sort_edges(i,4) < threshold(b,2) && sort_edges(i,3) < threshold(a,1) && sort_edges(i,3) < threshold(b,1))
                    if(sort_edges(i,4) < threshold(a,2) && sort_edges(i,4) < threshold(b,2))
                        u.join(a,b);
                        a = u.find(a);
                        threshold(a,2) = sort_edges(i,4) + ((g.n)/u.size(a));
                    end
                else
                    if(sort_edges(i,g.method) < threshold(a,g.method-2) && sort_edges(i,g.method) < threshold(b,g.method-2))
                        u.join(a,b);
                        a = u.find(a);
                        threshold(a,g.method-2) = sort_edges(i,g.method) + ((g.nnc)/u.size(a));
                    end
                end
            end
        end
    end
    toc

    %% enforce min zie
    %{
    disp('======= Enforce miz_size  =======');
    tic

    for i=1:1:g.nedges
       a = u.find(sort_edges(i,1));
       b = u.find(sort_edges(i,2));
       if(sort_edges(i,4) ~= g.NO_EDGE)
           if ((a ~= b) && (u.size(a) < g.min_size) || (u.size(b) < g.min_size)) 
               u.join(a,b);
           end
       end
    end
        toc
    %}

    for i=1:1:g.nnodes
       if nodes_in(i,1) ~= 0
           nodes_out(i,1) = u.find(i);
       else
           nodes_out(i, 1) = 0;
       end
    end
        g.num_ccs = u.num_sets();
end