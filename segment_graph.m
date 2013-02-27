function [nodes, g] = segment_graph(nodes, edges, g)
    tic
    sort_edges = sortrows(edges, 4);
    
    threshold = zeros(g.nnodes, 2);
    idd = 1:1:g.nnodes;
%     maxr = max(nodes(idd,4));
%     minr = min(nodes(idd,4));
    threshold(idd,1) = g.c*((nodes(idd,4))/4);
    % threshold(idd,2) = (g.n)./(nodes(idd,4)/10);
    threshold(idd,2) = (g.n);
    u = universe;
    u.initialize(g.nnodes);
    for i=1:1:g.nedges
        a = u.find(sort_edges(i,1));
        b = u.find(sort_edges(i,2));
        if(sort_edges(i,3) ~= g.NO_EDGE)
            if (a ~= b)
    %             if(sort_edges(i,3) < threshold(a,1) && sort_edges(i,3) < threshold(b,1))
    %                 u.join(a,b);
    %                 a = u.find(a);
    %                 threshold(a,1) = sort_edges(i,3) + ( g.c/u.size(a));
    %             end
                if(sort_edges(i,4) < threshold(a,2) && sort_edges(i,4) < threshold(b,2) && sort_edges(i,3) < threshold(a,1) && sort_edges(i,3) < threshold(b,1))
                    u.join(a,b);
                    a = u.find(a);
                    threshold(a,2) = sort_edges(i,4) + ((g.n)/u.size(a));
    %                 threshold(a,1) = sort_edges(i,3) + ( g.c/u.size(a));
                end
            end
        end
    end
    toc

    %% enforce min zie
%     %{
    disp('======= Enforce miz_size  =======');
    tic

    for i=1:1:g.nedges
       a = u.find(sort_edges(i,1));
       b = u.find(sort_edges(i,2));
       if(sort_edges(i,3) ~= g.NO_EDGE)
           if ((a ~= b) && (u.size(a) < g.min_size) || (u.size(b) < g.min_size)) && sort_edges(i,3) < threshold(a,1) && sort_edges(i,3) < threshold(b,1)
               u.join(a,b);
           end
       end
    end
        toc
%     %}
    g.num_ccs = u.num_sets();
    
    for i=1:1:g.nnodes
       if nodes(i,4) ~= 0
           nodes(i,8) = u.find(i);
       else
           nodes(i, 8) = 0;
       end
    end
end