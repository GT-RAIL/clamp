function dataset = generateObstacleDataset2D(center_x, center_y, w, h)
%GENERATEOBSTACLEDATASET2D Generates 2D dataset evidence grid
%   
%   Input Format:
%   center_x           (vector) x coordinate of the obstacle center
%   center_y           (vector) y coordinate of the obstacle center
%   l                  (vector) length
%   w                  (vector) width
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
dataset.cols = 500; %x
dataset.rows = 400; %y
dataset.origin_x = -25;
dataset.origin_y = -20;
dataset.cell_size = 0.1;
% map
dataset.map = zeros(dataset.rows, dataset.cols);

num_obs = size(center_x, 1);
% obstacles

for ii=1:num_obs
	dataset.map = add_obstacle(get_center(center_x(ii),center_y(ii),dataset), get_dim(w(ii), h(ii), dataset), dataset.map);
end

end


function [map, landmarks] = add_obstacle(position, size, map, landmarks, origin_x, origin_y, cell_size)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col) ...
    = ones(2*half_size_row+1, 2*half_size_col+1); 

% landmarks
if nargin == 7
    for x = position(1)-half_size_row-1 : 4 : position(1)+half_size_row-1
        y = position(2)-half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        y = position(2)+half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
    
    for y = position(2)-half_size_col+3 : 4 : position(2)+half_size_col-5
        x = position(1)-half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        x = position(1)+half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
end

end

function center = get_center(x,y,dataset)

center = [y - dataset.origin_y, x - dataset.origin_x]./dataset.cell_size;

end

function dim = get_dim(w,h,dataset)

dim = [h, w]./dataset.cell_size;

end