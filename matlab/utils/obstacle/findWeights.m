function [WTrain, ObsDistTrain] = findWeights(xTrain, sdfTrain, obsEpsilon, obsSigma)

import gpmp2.*
import gtsam.*

numPts = length(xTrain) + 1;
numDim = size(xTrain{1},3);
numDemos = size(xTrain{1},2);

% obsDistTrain = zeros(numDemos, numDemos, numPts-1);
% wTrain = zeros(numDemos, numDemos, numPts-1);


WTrain = cell(1, numPts-1);
ObsDistTrain = cell(1, numPts-1);
for n = 1:(numPts-1)
    obsWeight = zeros(numDemos,1);
    for i = 1:numDemos
        pos = squeeze(xTrain{n}(2, i, 1:numDim));
        
        if numDim == 2
            obstacleDist = sdfTrain(i).getSignedDistance(Point2(pos(1), pos(2)));
        elseif numDim == 3
            obstacleDist = sdfTrain(i).getSignedDistance(Point3(pos(1), pos(2), pos(3)));
        else
            error('invalid number of dimensions!')
            return
        end
        
        obstacleFreeCost = (obstacleDist > obsEpsilon)*0 + (obstacleDist <= obsEpsilon)*(-obstacleDist + obsEpsilon);
        
        obstacleFreeLikelihood = exp(-0.5*obstacleFreeCost.^2/obsSigma);
        
        obsDistVec(i) = obstacleDist*(obstacleDist>=0) + 0*(obstacleDist<0); 
        obsWeight(i) = obstacleFreeLikelihood;
    end
    
    WTrain{n} = diag(obsWeight);
    ObsDistTrain{n} = diag(obsDistVec);
end




end
