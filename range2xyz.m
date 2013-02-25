function p = range2xyz(r, yaw, pitch)
    p = zeros(3,1);
    cos_pitch = cosd(pitch);
    sin_pitch = sind(pitch);
    cos_yaw = cosd(yaw);
    sin_yaw = sind(yaw);
    
    p(1,1) = r .* cos_pitch .* cos_yaw;
    p(2,1) = r .* cos_pitch .* sin_yaw;
    p(3,1) = r .* sin_pitch;
end