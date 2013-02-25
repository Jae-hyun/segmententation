function plot_nodes(nodes, nstart, nend)
    figure;
    plot3(nodes(:,1), nodes(:,2),nodes(:,3)*-1, 'r.');
    axis equal;
    hold on;
    % colormap hsv
    % imshow(img);

    x1 = nodes(nstart:nend,1);
    % x = x(x~=0);
    y1 = nodes(nstart:nend,2);
    % y = y(y~=0);
    z1 = nodes(nstart:nend,3)*-1;
    nx1 = nodes(nstart:nend,5);
    ny1 = nodes(nstart:nend,6);
    nz1 = nodes(nstart:nend,7);
    % z = z(z~=0);
    idnzero = ~(x1 == 0 & y1 == 0);
    x = x1(idnzero);
    y = y1(idnzero);
    z = z1(idnzero);
%     nx2 = nx1(idnzero);
%     ny2 = ny1(idnzero);
%     nz2 = nz1(idnzero);
    nx = smooth(nx1(idnzero));
    ny = smooth(ny1(idnzero));
    nz = smooth(nz1(idnzero));
    quiver3(x,y,z,nx,ny,nz,0.2);

    figure;
    plot3(x1, y1,z1, 'r.');
    hold on;
    axis equal;
    quiver3(x1,y1,z1,nx1,ny1,nz1, 0.2);
%     figure(3);
% %     hold on;
%     axis equal;
%     plot3(x, y, z, 'r.');
%{
    % figure;
    % 
    % hold on;
    % plot(nx2, 'b');
    % plot(nx, 'r');

    % % Start Mesh Grid
    % dx=0.1;
    % dy=0.1;
    % 
    % x_edge=[floor(min(x)):dx:ceil(max(x))];
    % y_edge=[floor(min(y)):dy:ceil(max(y))];
    % [X,Y]=meshgrid(x_edge,y_edge);
    % 
    % % Z=griddata(x,y,z,X,Y);
    % % F=TriScatteredInterp(x,y,z);
    % % F = DelaunayTri(x, y);
    % % The following line of code is if you use JE's gridfit:
    % % Z=gridfit(x,y,z,x_edge,y_edge);
    % % Z = F(X,Y);
    % 
    % 
    % %NOW surf and mesh will work...
    % 
    % % surf(X,Y,Z)
    % % meshc(X,Y,Z)
    % %End of code to be copied-----------------------
%}
end