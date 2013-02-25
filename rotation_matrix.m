function R = rotation_matrix(yaw, pitch)

  c_a = cosd( yaw );
  s_a = sind( yaw );
  c_b = cosd( pitch );
  s_b = sind( pitch );

%   R.resize( 3, 3 ); //(row,col)

  R(1, 1) = c_a * c_b;
  R(2, 1) = s_a * c_b;
  R(3, 1) = -s_b;

  R(1, 2) = -s_a;
  R(2, 2) = c_a;
  R(3, 2) = 0.0;

  R(1, 3) = c_a * s_b;
  R(2, 3) = s_a * s_b;
  R(3, 3) = c_b;
end