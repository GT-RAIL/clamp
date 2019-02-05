function [XTrain, YTrain, sdfTrain] = setupTrainingLR(dataset)

import gtsam.*
import gpmp2.*
import gplfd.*

numDemos = size(dataset,1);
numDim = size(dataset(1).pos,2);
numStates = 2;
numPts = size(dataset(1).pos,1);

%% restructure training data

YTrain = cell(1, numPts-1);
XTrain = cell(1, numPts-1);

for n = 1:(numPts-1)
    YTrain{n} = zeros(numStates, numDemos, numDim);
    XTrain{n} = zeros(numStates+1, numDemos, numDim);
    for d = 1:numDim
        for i = 1:numDemos
            YTrain{n}(:, i, d) = [(dataset(i).pos(n+1,d)); (dataset(i).vel(n+1,d))];
            XTrain{n}(:, i, d) = [1; (dataset(i).pos(n,d)); (dataset(i).vel(n,d))];   %NOTE: 1 is appended in the beginning.
        end
    end
end

%% Generate sdf

for i = 1:numDemos
    if ~isfield(dataset(i),'obs')
        dataset(i).obs = [];
    end
    
    if numDim==2
        [sdfTrain(i), ~] = setupEnvironment2D(dataset(i).obs);
    elseif numDim==3
        [sdfTrain(i), ~] = setupEnvironment3D(dataset(i).obs);
    else
        error('num of dims invalid!!')
        return
    end
    
end

end