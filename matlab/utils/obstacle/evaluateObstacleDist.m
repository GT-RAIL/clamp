function dist = evaluateObstacleDist(field, x, y, origin_x, origin_y, cell_size)

grid_cols = floor((x - origin_x)./cell_size) + 1;
grid_rows = floor((y - origin_y)./cell_size) + 1;

dist = zeros(size(x));
for i=1:length(grid_rows)
    dist(i) = field(grid_rows(i), grid_cols(i));
end

end