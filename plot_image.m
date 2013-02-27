function plot_image(deci_img,nodes, g)
    temp_img = zeros(g.nscans, g.nranges);
%     cm = colormap(jet(100));
    for s = 0:1:g.nscans-1
       for r=0:1:g.nranges-1
           idx = s* g.nranges + r;
           temp_img(s+1,r+1) = mod(nodes(idx+1,1), 100);
       end
    end
    f = figure;
    fullscreen = get(0,'ScreenSize');
    set(f, 'Position',[fullscreen(3)/5, fullscreen(4)/4, 870*1.5, 128*4]);
    subplot(2,1,1);
    image(deci_img);
    axis image;
    grid on;
    %# grid domains
    xg = 0:g.nranges/4:g.nranges;
    yg = 0:11:g.nscans;
    %# label coordinates
    [xlbl, ylbl] = meshgrid(xg+10, yg+5);
    %# create cell arrays of number labels
    lbl = strtrim(cellstr(num2str((1:numel(xlbl))')));
    text(xlbl(:), ylbl(:), lbl(:),'color','w',...
        'HorizontalAlignment','center','VerticalAlignment','middle');
    subplot(2,1,2);
    image(temp_img);
    colormap(jet(100));
end