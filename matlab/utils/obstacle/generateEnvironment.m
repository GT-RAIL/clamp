function sdf = generateEnvironment(obs_info)

import gtsam.*
import gpmp2.*
import clamp.*

if isempty(obs_info)
    obs_dataset = generateObstacleDataset3D([],[],[],[]);
else
    obs_dataset = generateObstacleDataset3D(obs_info(:,1), obs_info(:,2), obs_info(:,3), obs_info(:,4), obs_info(:,5), obs_info(:,6));
end

cell_size = obs_dataset.cell_size;
origin = [obs_dataset.origin_x, obs_dataset.origin_y, obs_dataset.origin_z];
origin_point3 = Point3(origin');

% signed distance field
field = signedDistanceField3D(obs_dataset.map, cell_size);
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), size(field, 2), size(field, 3));

for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end


end