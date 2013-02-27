function [nodes] = read_image_to_nodes(img_matrix, g)
g.yaw_resolution = (g.img_yaw_end-g.img_yaw_start)/g.nranges;
vert_resolution = g.deci_s;
yaw_angle = g.img_yaw_start:g.yaw_resolution:g.img_yaw_end;
vert_angle = [-1.9367; -1.57397; -1.30476; -0.871566; -0.57881; -0.180617; 0.088762; 0.451829; 0.80315; 1.20124; 1.49388; 1.83324; 2.20757; 2.54663; 2.87384; 3.23588; 3.53933; 3.93585; 4.21552; 4.5881; 4.91379; 5.25078; 5.6106; 5.9584; 6.32889; 6.67575; 6.99904; 7.28731; 7.67877; 8.05803; 8.31047; 8.71141; 9.02602; 9.57351; 10.0625; 10.4707; 10.9569; 11.599; 12.115; 12.5621; 13.041; 13.4848; 14.0483; 14.5981; 15.1887; 15.6567; 16.1766; 16.554; 17.1868; 17.7304; 18.3234; 18.7971; 19.3202; 19.7364; 20.2226; 20.7877; 21.3181; 21.9355; 22.4376; 22.8566; 23.3224; 23.971; 24.5066; 24.9992];
nodes = zeros(g.nnodes, 4);

for s = 0:1:g.nscans-1
   for r=0:1:g.nranges-1
      idx = s* g.nranges + r;
%       nodes(idx+1,1) = img_matrix(s+1, r+1); 
      nodes(idx+1, 1:3) = range2xyz(img_matrix(s+1,r+1), yaw_angle(r+1), vert_angle(floor(s*vert_resolution)+1));
      nodes(idx+1, 4) = img_matrix(s+1,r+1);
   end
end
end