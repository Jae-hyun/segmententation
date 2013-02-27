function [deci_img, g] = down_sample_img(img_matrix, g)
    g.img_yaw_start = (360/g.nranges) * g.img_scan_start;
    g.img_yaw_end = (360/g.nranges) * g.img_scan_end;
    img_matrix = img_matrix(:, g.img_scan_start:g.img_scan_end);
    [g.nscans g.nranges] = size(img_matrix);
    deci_s = g.deci_s;
    deci_r = g.deci_r;
    ndscans = floor(g.nscans/deci_s);
    ndranges = floor(g.nranges/deci_r);
    deci_img = zeros(ndscans, ndranges);
    for s = 1:1:ndscans
       for r=1:1:ndranges
          deci_img(s, r) = img_matrix(floor(s*deci_s), floor(r*deci_r));
       end
    end

    [g.nscans g.nranges] = size(deci_img);
    
%     g.nranges = ndranges;
end