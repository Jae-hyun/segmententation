function plot_graph(nodes, edges, g)
    test_edges = zeros(g.nedges,3);   
    size_test_edges = 1;
    for i=1:1:g.nedges
        if edges(i,3) ~= g.NO_EDGE && isnan(edges(i,3)) ~= 1
            test_edges(size_test_edges,:) = edges(i,:);
            size_test_edges = size_test_edges + 1;
        end
    end
    %%
    xx1 = zeros(2, size_test_edges-1);
    yy1 = zeros(2, size_test_edges-1);
    zz1 = zeros(2, size_test_edges-1);
    for i=1:1:size_test_edges-1
        x1 = nodes(test_edges(i,1),1)*-1;
        y1 = nodes(test_edges(i,1),2);
        z1 = nodes(test_edges(i,1),3)*-1;
        x2 = nodes(test_edges(i,2),1)*-1;
        y2 = nodes(test_edges(i,2),2);
        z2 = nodes(test_edges(i,2),3)*-1;
        xx1(1:2,i) = [x1 ; x2];
        yy1(1:2,i) = [y1 ; y2];
        zz1(1:2,i) = [z1 ; z2];
    end
    %%
        figure;hold on;axis equal;
        plot3(xx1, yy1, zz1, 'r.');
%         plot3(nodes(:,1), nodes(:,2),nodes(:,3)*-1, 'r.');
        plot3(xx1, yy1, zz1, 'color', 'b', 'linestyle', '-');
end