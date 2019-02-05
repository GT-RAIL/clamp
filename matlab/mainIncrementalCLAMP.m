% Example script which demonstrates 
%   1) Trajectory prior learning in Incremental mode 
%   2) Trajectory reproduction in 2 new environments
% @author Muhammad Asif Rana

close all; clear; clc;
addpath(genpath('utils/'))

%% User-defined params

datasetName = 'data/box_closing.mat';

% arm settings (the data should be consisent with the arm you select):
armName = 'JACO2Arm';       % name of the arm used to record demonstrations (config space factors depend on this)
negateRootJoint = 1;        % depending on the DH parameters convention you use, you may have to change the sign of the first joint (NOT ALWAYS REQUIRED)

% prior learning param:
lambdaRidge = 1e-10;                % regularization coeff for MLE estimate

% reproduction environment params:

% Start/Goal Configurations to initialize optimization. The optimization
% initializes froma straight-line trajectory between the two configurations
startConfInitialization = [1.24, 2.70, 5.26, -1.10, -0.09, -1.83]';     
goalConfInitialization = [0.39, 2.32, 5.20, -1.05, -0.40, -1.83]';

% Specify new start/goal positions and obstacles 
%(Be careful not to specify conflictin init/goal)
startPositionWorkspace = [0.2054, -0.4722, 0.2625]';    % new initial position in workspace
startPostionConfigspace = [];                           % new initial position in configuration space    
goalPositionWorkspace = [];                             % new goal position in workspace
goalPositionConfigspace = [];                           % new goal position in configuration space
obstacleInfo = [0.32, -0.32, 0.13, 0.09, 0.06, 0.28];   % obstacle information: each row is in the form: [center_x, center_y, center_z, l, w, h]

%% Loading data
disp('Loading data')

load(datasetName);

h1 = plotDataset3D(dataset); view(34,32); axis tight; set(h1, 'units','normalized','outerposition', [0.0245,0,0.4875,0.9009]);
set(h1, 'name','Human Demonstrations visualized in 3D');
title('Human Demonstrations in 3D');
h2 = plotDatasetTime(dataset); set(h2, 'units','normalized','outerposition',[0.51,0,0.4875,0.9009]);
set(h2,'name','Human Demonstrations against time');
disp('Visualizing dataset ... Press any key to continue');
pause;

numDim = size(dataset(1).pos,2);    % no of dimensions
numDemos = size(dataset,1);         % no of demonstrations in the dataset
numPts = size(dataset(1).pos,1);    % no of trajectory points
numStates = 2;                      % order of the dynamical system (DO NOT CHANGE THIS)

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

[XTrain, YTrain] = setupTrainingLR(dataset);

%% Setting up priors 
disp('Incremental Learning of the trajectory prior')

% setting up prior of dynamics learning
MPrior = cell(1, numPts-1);
KPrior = cell(1, numPts-1);
SPrior = cell(1, numPts-1);
NPrior = cell(1, numPts-1);

dt = dataset(1).time(2) - dataset(1).time(1);

N0 = 1e-10;
for d = 1:numDim
    for n=1:numPts-1
        MPrior{n}(:,:,d) = [0, 0, 0; 0, 0, 0];
        KPrior{n}(:,:,d) = 1e-10*eye(numStates+1); 
        NPrior{n}(1,d) = N0;
        SPrior{n}(:,:,d) = N0*eye(numStates);
    end
end

%% Carry out Incremenal Learning of Trajectory Prior and Visualizing

MPosterior = MPrior;
KPosterior = KPrior;
SPosterior = SPrior;
NPosterior = NPrior;

colorPrior = [0 0 1];
for n = 1:numDemos
    xTrain = cellfun(@(x) x(:,n,:), XTrain, 'UniformOutput', false);
    yTrain = cellfun(@(x) x(:,n,:), YTrain, 'UniformOutput', false);
    
    [MPosterior, KPosterior, SPosterior, NPosterior, Vmode] = updateParamsBLR(MPosterior, KPosterior, SPosterior, NPosterior, xTrain, yTrain);
    
    [PhiCell, NuCell, QCell] = convertParams2Dynamics(MPosterior, Vmode);
    [MuPrior, KPrior, PhiMat, NuMat, QMat] = convertDynamics2TrajPrior(PhiCell, NuCell, QCell, Nu0, Q0);
    
    if n == ceil(numDemos/3)
        h = plotGPTime(dataset(1).time, MuPrior, KPrior, numDim, numStates, numPts, colorPrior);
        set(h, 'units','normalized','outerposition',[0 0 1 1]);
        set(h,'name',['Trajectory prior after ', int2str(n), ' iterations']);
        disp(['Visualizng Trajectory prior after ', int2str(n), ' iterations']);
        pause(1.5)
        
    elseif n == 2*ceil(numDemos/3)
        h = plotGPTime(dataset(1).time, MuPrior, KPrior, numDim, numStates, numPts, colorPrior);
        set(h, 'units','normalized','outerposition',[0 0 1 1]);
        set(h,'name',['Trajectory prior after ', int2str(n), ' iterations']);
        disp(['Visualizng Trajectory prior after ', int2str(n), ' iterations']);
        pause(1.5)
        
    elseif n == numDemos
        h = plotGPTime(dataset(1).time, MuPrior, KPrior, numDim, numStates, numPts, colorPrior);
        set(h, 'units','normalized','outerposition',[0 0 1 1]);
        set(h,'name',['Trajectory prior after ', int2str(n), ' iterations']);
        disp(['Visualizng Trajectory prior after ', int2str(n), ' iterations']);
        pause(1.5);
    end
end


disp('Plotting the final trajectory prior in 3D')
colorPrior = [0 0 1];
h4 = figure; hold on; set(h4, 'units','normalized','outerposition',[0 0 1 1]); hold on; grid on; box on; axis equal; axis tight; view(52,5); 
% plotDataset3D(dataset, h4, [150 150 150]./255, 0);
plotGPXYZ(MuPrior, KPrior, numDim, numStates, numPts, colorPrior, 1);
title('Trajectory Prior in 3D')
set(h4,'name','3D visualization of trajectory prior');
disp('Press any key to continue ... ');
pause;


priorParams.Phi = PhiCell;
priorParams.Q = QCell;
priorParams.Nu = NuCell;
priorParams.Nu0 = Nu0;
priorParams.Q0 = Q0;
priorParams.dt = dt;
priorParams.numPts = numPts;
priorParams.dataset = dataset;

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
optSettings.goalConfInit = goalConfInitialization ;
optSettings.dof = optSettings.armModel.dof();
optSettings.Qc = 1;
optSettings.fixSigma = 1e-6;
optSettings.maxIterations = 500;
optSettings.plot = 1;
optSettings.animate = 0;

%% Optimization 
traj_empty_env = reproduceTraj(priorParams, likelihoodParams, optSettings);
h5 = gcf;
plot3(startPositionWorkspace(1),startPositionWorkspace(2),startPositionWorkspace(3), 'r-*', 'MarkerSize', 10, 'LineWidth',5, 'MarkerFaceColor',[.49 1 .63], 'MarkerEdgeColor','k');
title('Trajectory Reproduction from a New Initial Position')
set(h5, 'units','normalized','outerposition',[0.0245,0,0.4875,0.9009]);
set(h5,'name', 'Trajectory Repoduction from New Start Position')

%% Adding a new obstacle
[likelihoodParams.obsSDF, likelihoodParams.obsDataset] = setupEnvironment3D(obstacleInfo);

%% Optimization
traj_obs_env = reproduceTraj(priorParams, likelihoodParams, optSettings);
h6 = gcf;
plot3(startPositionWorkspace(1),startPositionWorkspace(2),startPositionWorkspace(3), 'r-*', 'MarkerSize', 10, 'LineWidth',5, 'MarkerFaceColor',[.49 1 .63], 'MarkerEdgeColor','k');
title('Trajectory Reproduction from a New Initial Pos w/ New Obstacle')
set(h6, 'units','normalized','outerposition',[0.51,0,0.4875,0.9009]);
set(h6,'name', 'Trajectory Repoduction from New Start Position w/ Obstacle')

