function plot_image(deci_img,nodes,grad_img, g)
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
    set(f, 'Position',[fullscreen(3)/6, 0, 870*1.5, 128*6]);
    subplot(5,1,1);
        colormap(jet(100));
    imagesc(deci_img);

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
    subplot(5,1,2);
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
    subplot(5,1,3);
%         colormap bone;
    image(temp_img1);
    
    subplot(5,1,4);
%         colormap bone;
    imagesc(grad_img.*180/3.14);
    temp_grad = grad_img.*180/3.14;
%     myfilter = fspecial('gaussian',[3 3], 0.5);
%     temp_grad = filter2(myfilter, temp_grad);
    for s = 1:1:g.nscans
        for r=1:1:g.nranges
            if (temp_grad(s, r) <= -80) && temp_grad(s, r) >= -100
                temp_grad(s, r) = -180;
%             elseif (temp_grad(s, r) <= -80) && temp_grad(s, r) >= -90
%                 temp_grad(s, r) = 180;
            end
        end
    end
%     figure;
    subplot(5,1,5);
    imagesc(temp_grad);
    
    figure;
    [t_m, t_d] = imgradient(temp_grad);
    imagesc(t_m);
%     tic
%     [m_i, d_i] = imgradient(temp_img1);
%     toc
%     figure;
%     subplot(2,1,1);
%     imagesc(m_i);
%     subplot(2,1,2);
%     imagesc(d_i);
    

end