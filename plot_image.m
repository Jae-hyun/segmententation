function plot_image(deci_img,nodes,grad_img, g, mog_img)
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
    % fullscreen = [x_start y_start x_end y_end]
    set(f, 'Position',[fullscreen(3)/2, 0, fullscreen(3)/2, fullscreen(4)]);
    subplot(5,1,1);
        colormap(jet(100));
    imagesc(deci_img);
    title('Original Range Image');

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
    title('Graph Based Segmentation Result Extracting Ground');

%     axis image;
    grid on;   
    
    
    temp_img1 = zeros(g.nscans, g.nranges);
%     cm = colormap(jet(100));
    for s = 0:1:g.nscans-1
       for r=0:1:g.nranges-1
           idx = s* g.nranges + r;
           if nodes(idx+1,4) ~= 0
%                temp_img1(s+1,r+1) = (nodes(idx+1,5)*nodes(idx+1,5)+nodes(idx+1,6)*nodes(idx+1,6)+nodes(idx+1,7)*nodes(idx+1,7))*180/3.14;
               temp_img1(s+1,r+1) = nodes(idx+1,5)+nodes(idx+1,6)+nodes(idx+1,7);
%                temp_img1(s+1,r+1) = nodes(idx+1,7);
%                 temp_img1(s+1,r+1) = floor(acos(dp).*180/3.14);
%                dp = n1.x*n2.x + n1.y*n2.y + n1.z*n2.z
%                This will give a value in [-1 to 1].
%                If dp is negative, the angle is greater than 90 degrees.
%                
%                To find the angle, use arc-cosine.
%                
%                �� = acos(dp);
               %             temp_img(s+1,r+1) = nodes(idx+1,1);
           end
       end
    end
%     f1 = figure;
%     fullscreen = get(0,'ScreenSize');
%     set(f1, 'Position',[fullscreen(3)/5, fullscreen(4)/4, 870*1.5, 128*4]);
    subplot(5,1,3);
%         colormap bone;
    % Normal Image
%     image(temp_img1);
    % Magnitude of Gradient Image
    image(mog_img);
    title('MoD Image');
    
    subplot(5,1,4);
%         colormap bone;
%     imagesc(abs(grad_img.*180/3.14));
    imagesc(grad_img.*180/3.14);
    title('DoG Image');
%     temp_grad = abs(grad_img.*180/3.14);
    temp_grad = grad_img.*180/3.14;
%     myfilter = fspecial('gaussian',[3 3], 0.5);
%     temp_grad = filter2(myfilter, temp_grad);
    for s = 1:1:g.nscans
        for r=1:1:g.nranges
            if (temp_grad(s, r) <= -80) && temp_grad(s, r) >= -100 || temp_grad(s, r)== 0
%             if (temp_grad(s, r) >= 80) && temp_grad(s, r) <= 100
                temp_grad(s, r) = -180;
                mog_img(s, r) = 0;
%             elseif (temp_grad(s, r) <= -80) && temp_grad(s, r) >= -90
%                 temp_grad(s, r) = 180;
            else
                temp_grad(s, r) = 0;
            end
        end
    end
%     figure;
    subplot(5,1,5);
    image(temp_img1);
    title('Ground Image using DoG');
    
    f1 = figure;
    set(f1, 'Position',[fullscreen(1), 0, fullscreen(3)/2, fullscreen(4)]);
    subplot(5,1,1);
    image(deci_img);
    title('Original Range Image');
    subplot(5,1,2);
    image(temp_img);
    title('Graph Based Segmentation Result Extracting Ground');

%     figure;
    [t_m, t_d] = imgradient(grad_img.*180/3.14);
    subplot(5,1,3);
    image(t_m);
    title('MDoG Image');
%     imagesc(t_m);
%     tic
%     [m_i, d_i] = imgradient(temp_img1);
%     toc
%     figure;
%     subplot(2,1,1);
%     image(t_m);
%     subplot(2,1,2);
%     imagesc(t_d);
    
    temp_grad1 = abs(t_m);
%     myfilter = fspecial('gaussian',[3 3], 0.5);
%     temp_grad = filter2(myfilter, temp_grad);
    for s = 1:1:g.nscans
        for r=1:1:g.nranges
            if temp_grad1(s, r) <= 20
%                 if temp_grad(s, r) == -180
                    temp_grad1(s, r) = -180;    
%                 else
%                     temp_grad1(s, r) = 0;
%                 end
            else
                temp_grad1(s, r) = 0;
            end
        end
    end
%     figure;
    subplot(5,1,4);
    imagesc(temp_grad);
    title('Ground Image using DoG');
    subplot(5,1,5);
    imagesc(temp_grad1);
    title('Ground Image using MDoG');
%     figure;
%     image(mog_img.*100);
%     [t_m, t_d] = imgradient(t_m);
%         figure;
%     subplot(2,1,1);
%     image(t_m);
%     subplot(2,1,2);
%     imagesc(t_d);
    figure;
    imagesc(temp_img1.*180/3.14);
end