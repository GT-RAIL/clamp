% Example script which demonstrates skill learning from demonstrations
% which are influenced by the presence of obstacles in the environment.
% @author Muhammad Asif Rana


close all; clear; clc;
addpath(genpath('utils/'))

%% User-defined params

datasetName = 'data/reaching_cluttered.mat';

% arm settings (the data should be consisent with the arm you select):
armName = 'JACO2_7';       % name of the arm used to record demonstrations (config space factors depend on this)
negateRootJoint = 1;        % depending on the DH parameters convention you use, you may have to change the sign of the first joint (NOT ALWAYS REQUIRED)

% prior learning param:
lambdaRidge = 1e-10;                % regularization coeff for MLE estimate

obstacleWeightEpsilon = 0.3;        % distance around obstacle with influence (danger region)
obstacleWeightSigma = 0.001;        % the rate of increase of the influence inside the danger region


% reproduction environment params:
% Start/Goal Configurations to initialize optimization. The optimization
% initializes froma straight-line trajectory between the two configurations
startConfInitialization = [3.60;3.69;2.51;4.55;3.58;4.09;2.49];
goalConfInitialization = [4.80;4.12;3.12;4.35;2.74;3.75;2.34];

% Specify new start/goal positions and obstacles 
%(Be careful not to specify conflictin init/goal)
startPositionWorkspace = [0.20;-0.10;0.20];    % new initial position in workspace
startPostionConfigspace = [];                           % new initial position in configuration space    
goalPositionWorkspace = [];                             % new goal position in workspace
goalPositionConfigspace = [];                           % new goal position in configuration space
obstacleInfo = [];   % obstacle information: each row is in the form: [center_x, center_y, center_z, l, w, h]

%% Loading data
disp('Loading data')

load(datasetName);

h1 = plotDataset3D(dataset); view(34,32); axis tight; set(h1, 'units','normalized','outerposition',[0 0 1 1]);
set(h1, 'name','Human Demonstrations visualized in 3D'); hold on;
title('Human Demonstrations in 3D in Clutter');
plotEnvironment(dataset(1).obs);
pause;

numDim = size(dataset(1).pos,2);    % no of dimensions
numDemos = size(dataset,1);         % no of demonstrations in the dataset
numPts = size(dataset(1).pos,1);    % no of trajectory points
numStates = 2;                      % order of the dynamical system (DO NOT CHANGE THIS)
dt = dataset(1).time(2) - dataset(1).time(1);

x0 = zeros(numDemos,numStates*numDim);
xT = zeros(numDemos,numStates*numDim);
for i=1:numDemos
    x0(i,:) = [dataset(i).pos(1,:), dataset(i).vel(1,:)];
    xT(i,:) = [dataset(i).pos(end,:), dataset(i).vel(end,:)];
end
Nu0 = mean(x0).';
Q0 = cov(x0).' + 1e-12*eye(numDim*numStates);

NuT = mean(xT).';
QT = cov(xT).' + 1e-12*eye(numDim*numStates);

%% Setup Training Data
disp('Setting up training data')

[XTrain, YTrain, sdfTrain] = setupTrainingLR(dataset);

%% Carry out Weighted Linear Regression

disp('Finding obstacle influence weights');

[WTrain, ObsDistTrain] = findWeights(XTrain, sdfTrain, obstacleWeightEpsilon, obstacleWeightSigma);

disp('Learning unweighted dynamics parameters')

[APredicted, VPredicted] = updateParamsMLE(lambdaRidge,XTrain,YTrain);
[PhiCell, NuCell, QCell] = convertParams2Dynamics(APredicted, VPredicted);

disp('Learning weighted dynamics parameters')

[APredicted_weighted, VPredicted_weighted] = updateParamsMLE(lambdaRidge,XTrain,YTrain, WTrain);
[PhiCell_weighted, NuCell_weighted, QCell_weighted] = convertParams2Dynamics(APredicted_weighted, VPredicted_weighted);

%% Setting up the prior
disp('Setting up the prior ')

[MuPrior, KPrior, PhiMat, NuMat, QMat] = convertDynamics2TrajPrior(PhiCell, NuCell, QCell, Nu0, Q0);
[MuPrior_weighted, KPrior_weighted, PhiMat_weighted, NuMat_weighted, QMat_weighted] = convertDynamics2TrajPrior(PhiCell_weighted, NuCell_weighted, QCell_weighted, Nu0, Q0);

priorParams.Phi = PhiCell;
priorParams.Q = QCell;
priorParams.Nu = NuCell;
priorParams.Nu0 = Nu0;
priorParams.Q0 = Q0;
priorParams.dt = dt;
priorParams.numPts = numPts;
priorParams.dataset = dataset;

priorParams_weighted = priorParams;
priorParams_weighted.Phi = PhiCell_weighted;
priorParams_weighted.Q = QCell_weighted;
priorParams_weighted.Nu = NuCell_weighted;

%% Plotting Prior against time
disp('Plotting the prior... May take a few moments')
colorPrior = [0 0 1];
h4 = figure; hold on; set(h4, 'units','normalized','outerposition',[0 0 1 1]); hold on; 
set(h4,'name','3D visualization of trajectory prior');

subplot(1,2,1); hold on; title('Unweighted prior'); view(33,62); grid on; box on; axis equal; axis tight;  
plotGPXYZ(MuPrior, KPrior, numDim, numStates, numPts, colorPrior, 1);

subplot(1,2,2); hold on; title('Weighted prior'); view(33,62); grid on; box on; axis equal; axis tight;  
plotGPXYZ(MuPrior_weighted, KPrior_weighted, numDim, numStates, numPts, colorPrior, 1);

disp('Press any key to continue...');
pause;
%% Adding obstacle
disp('Setting up the Reproduction Environment')

likelihoodParams = struct();
likelihoodParams.obsEpsilon = 0.05;         % safety distance around obstacle where the obstacle effects the trajectories
likelihoodParams.obsSigma = 0.05;           % the rate of increase of the obstacle cost           
likelihoodParams.startConfFixed = startPostionConfigspace;
likelihoodParams.goalConfFixed = goalPositionConfigspace;
likelihoodParams.startCartFixed = startPositionWorkspace;
likelihoodParams.goalCartFixed = goalPositionWorkspace;
[likelihoodParams.obsSDF, likelihoodParams.obsDataset] = setupEnvironment3D([]);

%% Optimization Settings
disp('Setting up the Optimization')

optSettings = struct();
optSettings.negateRootJoint = negateRootJoint;
optSettings.eefModel = generateEEFLFD(armName);
optSettings.armModel = generateArmLFD(armName);
optSettings.startConfInit = startConfInitialization;
optSettings.goalConfInit = goalConfInitialization;
optSettings.dof = optSettings.armModel.dof();
optSettings.Qc = 1;
optSettings.fixSigma = 1e-6;
optSettings.maxIterations = 500;
optSettings.plot = 0;
optSettings.animate = 0;

%% Optimization 
traj_empty_env = reproduceTraj(priorParams, likelihoodParams, optSettings);
h5 = figure; hold on; set(h5, 'name','Trajectory reproduction in an empty environment');
set(h5, 'units','normalized','outerposition',[0 0 1 1]);

s1 = subplot(1,2,1); 
hold on; title('Reproduction from Unweighted Prior'); view(33,62); grid on; box on; axis equal; axis tight;  
plotDataset3D(priorParams.dataset, s1, [150 150 150]./255, 0);
plot3(traj_empty_env.pos(:,1), traj_empty_env.pos(:,2), traj_empty_env.pos(:,3), 'r', 'linewidth', 2);
plot3(startPositionWorkspace(1),startPositionWorkspace(2),startPositionWorkspace(3), 'r-*', 'MarkerSize', 10, 'LineWidth',5, 'MarkerFaceColor',[.49 1 .63], 'MarkerEdgeColor','k');

traj_empty_env_weighted = reproduceTraj(priorParams_weighted, likelihoodParams, optSettings);
s2 = subplot(1,2,2); 
hold on; title('Reproduction from Weighted Prior'); view(33,62); grid on; box on; axis equal; axis tight;  
plotDataset3D(priorParams_weighted.dataset, s2, [150 150 150]./255, 0);
plot3(traj_empty_env_weighted.pos(:,1), traj_empty_env_weighted.pos(:,2), traj_empty_env_weighted.pos(:,3), 'r', 'linewidth', 2);
plot3(startPositionWorkspace(1),startPositionWorkspace(2),startPositionWorkspace(3), 'r-*', 'MarkerSize', 10, 'LineWidth',5, 'MarkerFaceColor',[.49 1 .63], 'MarkerEdgeColor','k');


