function dataset = generateObstacleDataset3D(center_x, center_y, center_z, l, w, h)
%GENERATEOBSTACLEDATASET3D Generates 3D dataset evidence grid
%   
%   Input Format:
%   center_x           (vector) x coordinate of the obstacle center from (0,0,0)
%   center_y           (vector) y coordinate of the obstacle center from (0,0,0)
%   center_z           (vector) z coordinate of the obstacle center from (0,0,0)
%   l                  (vector) length
%   w                  (vector) width
%   h                  (vector) height
%
%
%   Output Format:
%   dataset.map        ground truth evidence grid
%   dataset.rows       number of rows (y)
%   dataset.cols       number of cols (x)
%   dataset.origin_x   origin of map x
%   dataset.origin_y   origin of map y
%   dataset.cell_size  cell size


% params
dataset.cols = 200;
dataset.rows = 200;
dataset.z = 200;
dataset.origin_x = -1;
dataset.origin_y = -1;
dataset.origin_z = -1;
dataset.cell_size = 0.01;

% map
dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
% obstacles
dataset.corner_idx = [];

num_obs = size(center_x, 1);

for ii = 1:num_obs
    [dataset.map, dataset.corner_idx] = add_obstacle(get_center(center_x(ii), center_y(ii), center_z(ii), dataset), get_dim(l(ii), w(ii), h(ii), dataset), dataset.map, dataset.corner_idx);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [map, corner] = add_obstacle(position, size, map, corner)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);
half_size_z = floor((size(3)-1)/2);

position = round(position);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col, ...
    position(3)-half_size_z   : position(3)+half_size_z) ...
    = ones(2*half_size_row+1, 2*half_size_col+1, 2*half_size_z+1); 

% corner
corner = [corner; ...
   [position(1)-half_size_row , position(1)+half_size_row, ...
    position(2)-half_size_col , position(2)+half_size_col,...
    position(3)-half_size_z   , position(3)+half_size_z]];


end


function center = get_center(x,y,z,dataset)

center = [x - dataset.origin_x, y - dataset.origin_y, z - dataset.origin_z]./dataset.cell_size;

end

function dim = get_dim(l,w,h,dataset)

dim = [l, w, h]./dataset.cell_size;

end
