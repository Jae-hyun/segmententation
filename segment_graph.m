function [nodes_out, g, sort_edges, threshold] = segment_graph(nodes_in, edges, g, labels)
    tic
    sort_edges = zeros(g.nedges,6);
%     if g.method == 6
        sort_edges = sortrows(edges, 3);
%         sort_edges = edges;
%     else
%         sort_edges = sortrows(edges, g.method);
%     end
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
%     for i=g.nedges:-1:1
        a = u.find(sort_edges(i,1));
        b = u.find(sort_edges(i,2));
        if(sort_edges(i,4) ~= g.NO_EDGE || sort_edges(i,4) ~= 0 && isnan(edges(i,3)) ~= 1)
            if (a ~= b)
                if g.method == 3
                    if(sort_edges(i,3) < threshold(a,1) && sort_edges(i,3) < threshold(b,1))
                        u.join(a,b);
                        a = u.find(a);
                        threshold(a,1) = sort_edges(i,3) + ( g.c/u.size(a));
                    end
                elseif g.method == 4
%                     if(g.n < 0.0 || (sort_edges(i,4) <= threshold(a,2)) && (sort_edges(i,4) <= threshold(b,2))) ...
%                       && (g.c < 0.0 || (sort_edges(i,3) <= threshold(a,1)) && (sort_edges(i,3) <= threshold(b,1)))
                    if(sort_edges(i,4) < threshold(a,2) && sort_edges(i,4) < threshold(b,2))
                        u.join(a,b);
                        a = u.find(a);
                        if g.n > 0.0
                            threshold(a,2) = sort_edges(i,4) + ((g.n)/u.size(a));
                        end
%                         if g.c > 0.0
%                             threshold(a,1) = sort_edges(i,3) + ( g.c/u.size(a));
%                         end
%                     else
%                         break;
                    end
                elseif g.method == 5
                    if(sort_edges(i,g.method) < threshold(a,3) && sort_edges(i,g.method) < threshold(b,3)) 
                        u.join(a,b);
                        a = u.find(a);
                        threshold(a,g.method-2) = sort_edges(i,g.method) + ((g.nnc)/u.size(a));
                    end
                elseif g.method == 6
                end
            end
%         else
%             break;
        end
    end
    toc
%     j = i;
%{
    i
    tic
    sort_edges1 = sortrows(sort_edges, 4);
    for j=1:1:g.nedges
        a = u.find(sort_edges1(j,1));
        b = u.find(sort_edges1(j,2));
        if(sort_edges1(j,3) ~= g.NO_EDGE)
            if (a ~= b) && ((u.size(a) < g.min_size) || (u.size(b) < g.min_size))
                if(sort_edges1(j,3) < threshold(a,1) && sort_edges1(j,3) < threshold(b,1))
                    u.join(a,b);
                    a = u.find(a);
                    threshold(a,1) = sort_edges1(j,3) + ( g.c/u.size(a));
                end
            end
        else
            break;
        end 
    end
     j

     toc
%}
    %% enforce min zie
    %{
    disp('======= Enforce miz_size  =======');
    tic

    for i=1:1:g.nedges
       a = u.find(sort_edges(i,1));
       b = u.find(sort_edges(i,2));
       if(sort_edges(i,3) ~= g.NO_EDGE) && (isnan(edges(i,3)) ~= 1)
           if ((a ~= b) && (u.size(a) < g.min_size) || (u.size(b) < g.min_size)) 
               u.join(a,b);
           end
%        else
%            break;
       end
    end
        toc
    %}
    for i=1:1:g.nedges
        sort_edges(i,6) = u.find(sort_edges(i,1));
    end
    g.num_ccs = u.num_sets();
    for i=1:1:g.nnodes
       if nodes_in(i,1) ~= 0 && labels(i,1) ~= g.label.ground
           p = u.find(i);
           if u.size(p) >= g.min_size
               nodes_out(i,1) = u.find(i);
           end
       else
           nodes_out(i, 1) = 0;
%            g.num_ccs = g.num_ccs - 1;
       end
    end
        
end