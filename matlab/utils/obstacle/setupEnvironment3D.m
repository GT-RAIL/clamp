function [sdf, obsDataset] = setupEnvironment3D(obsInfo)

import gtsam.*
import gpmp2.*
import clamp.*

if isempty(obsInfo)
    obsDataset = generateObstacleDataset3D([],[],[],[]);
else
    obsDataset = generateObstacleDataset3D(obsInfo(:,1), obsInfo(:,2), obsInfo(:,3), obsInfo(:,4), obsInfo(:,5), obsInfo(:,6));
end

cell_size = obsDataset.cell_size;
origin = [obsDataset.origin_x, obsDataset.origin_y, obsDataset.origin_z];
origin_point3 = Point3(origin');

% signed distance field
field = signedDistanceField3D(obsDataset.map, cell_size);
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), size(field, 2), size(field, 3));

for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

%plotMap3D(obs_dataset.corner_idx, origin, cell_size);

end
