function [g_mag, g_dir, g_x, g_y] = imgradient(deci_img)
[g_x, g_y] = gradient(deci_img);
% g = (abs(A)*2+abs(B)*1.5);
g_mag = sqrt(g_x.^2+g_y.^2);
% g_dir = atan2(B,A).*180/3.14;
% g_dir = atan2(g_y,g_x);
g_dir = atan2(g_y,g_x);
end