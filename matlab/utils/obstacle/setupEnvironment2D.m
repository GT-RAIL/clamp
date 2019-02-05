function [sdf, obsDataset] = setupEnvironment2D(obsInfo)

import gtsam.*
import gpmp2.*
import clamp.*

if isempty(obsInfo)
    obsDataset = generateObstacleDataset2D([],[],[],[]);
else
    obsDataset = generateObstacleDataset2D(obsInfo(:,1), obsInfo(:,2), obsInfo(:,3), obsInfo(:,4));
end

cell_size = obsDataset.cell_size;
origin_point2 = Point2(obsDataset.origin_x, obsDataset.origin_y);

% signed distance field
field = signedDistanceField2D(obsDataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);
% figure;
% plotCollisionFreeLikleihood2D(field,obs_dataset.origin_x, obs_dataset.origin_y,cell_size,2, 5.612)


end