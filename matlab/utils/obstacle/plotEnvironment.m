function plotEnvironment(obs_info)

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

plotMap3D(obs_dataset.corner_idx, [obs_dataset.origin_x, obs_dataset.origin_y, obs_dataset.origin_z], obs_dataset.cell_size);
        
end