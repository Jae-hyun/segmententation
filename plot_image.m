function plot_image(deci_img,nodes, g)
    temp_img = zeros(g.nscans, g.nranges);
%     cm = colormap(jet(100));
    ncolor = randperm(100);
    for s = 0:1:g.nscans-1
       for r=0:1:g.nranges-1
           idx = s* g.nranges + r;
           if nodes(idx+1,9) ~= 0
            temp_img(s+1,r+1) = ncolor(mod(nodes(idx+1,9), 100)+1);
           end
%             temp_img(s+1,r+1) = nodes(idx+1,1);
       end
    end
    f = figure;
    fullscreen = get(0,'ScreenSize');
    set(f, 'Position',[fullscreen(3)/5, fullscreen(4)/5, 870*1.5, 128*6]);
    subplot(3,1,1);
        colormap(jet(200));
    image(deci_img);

%     axis image;
    grid on;
%     hold off;
    %# grid domains
%     xg = 0:g.nranges/4:g.nranges;
%     yg = 0:11:g.nscans;
%     %# label coordinates
%     [xlbl, ylbl] = meshgrid(xg+10, yg+5);
%     %# create cell arrays of number labels
%     lbl = strtrim(cellstr(num2str((1:numel(xlbl))')));
%     text(xlbl(:), ylbl(:), lbl(:),'color','w',...
%         'HorizontalAlignment','center','VerticalAlignment','middle');
    subplot(3,1,2);
% figure;
        colormap(jet(100));
    image(temp_img);

%     axis image;
    grid on;   
    
    
    temp_img1 = zeros(g.nscans, g.nranges);
%     cm = colormap(jet(100));
    for s = 0:1:g.nscans-1
       for r=0:1:g.nranges-1
           idx = s* g.nranges + r;
           if nodes(idx+1,4) ~= 0
               temp_img1(s+1,r+1) = (nodes(idx+1,5)+nodes(idx+1,6)+nodes(idx+1,7))*60;
               %             temp_img(s+1,r+1) = nodes(idx+1,1);
           end
       end
    end
%     f1 = figure;
%     fullscreen = get(0,'ScreenSize');
%     set(f1, 'Position',[fullscreen(3)/5, fullscreen(4)/4, 870*1.5, 128*4]);
    subplot(3,1,3);
%         colormap bone;
    image(temp_img1);
    
    

end