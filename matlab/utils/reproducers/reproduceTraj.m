function trajRep = reproduceTraj(priorParams, likelihoodParams, optSettings)

import gtsam.*
import gpmp2.*
import clamp.*

%% copying parameters
Phi = priorParams.Phi;
Q = priorParams.Q;
Nu = priorParams.Nu;
Q0 = priorParams.Q0;
Nu0 = priorParams.Nu0;
numPts = priorParams.numPts;
dt = priorParams.dt;

obsSDF = likelihoodParams.obsSDF;
obsDataset = likelihoodParams.obsDataset;
obsEpsilon = likelihoodParams.obsEpsilon;
obsSigma = likelihoodParams.obsSigma;
startCartFixed = likelihoodParams.startCartFixed;
goalCartFixed = likelihoodParams.goalCartFixed;
startConfFixed = likelihoodParams.startConfFixed;
goalConfFixed = likelihoodParams.goalConfFixed;

negateRootJoint = optSettings.negateRootJoint;
startConfInit = optSettings.startConfInit;
goalConfInit = optSettings.goalConfInit;
armModel = optSettings.armModel;
eefModel = optSettings.eefModel;
dof = optSettings.dof;
fixSigma = optSettings.fixSigma;
Qc = optSettings.Qc;
maxIterations = optSettings.maxIterations;

if negateRootJoint
    if ~isempty(startConfInit)
        startConfInit(1) = -startConfInit(1);
    end
    
    if ~isempty(goalConfInit)
        goalConfInit(1) = -goalConfInit(1);
    end
    
    if ~isempty(startConfFixed)
        startConfFixed(1) = -startConfFixed(1);
    end
    
    if ~isempty(goalConfFixed)
        goalConfFixed(1) = -goalConfFixed(1);
    end    
end

%% settings

totalTimeStep = numPts-1;
QcMat = Qc*eye(dof);
QcModel = noiseModel.Gaussian.Covariance(QcMat);

poseFixModel = noiseModel.Isotropic.Sigma(dof, fixSigma);
cartFixModel = noiseModel.Isotropic.Sigma(3, fixSigma);

%% init optimization
initValues = initArmTrajStraightLine(startConfInit, goalConfInit, totalTimeStep);

%% Setup the graph
graph = NonlinearFactorGraph;
graph_obs = NonlinearFactorGraph;

for i = 0:totalTimeStep
    keyPos = symbol('x',i);
    keyVel = symbol('v',i);
    % start and goal prior
    if i==0
        if isa(eefModel, 'PointRobotModel')
            graph.add(GaussianPriorCartesianLinearPointRobot(keyPos, keyVel, eefModel, Nu0, Q0));
        else
            graph.add(GaussianPriorCartesianLinearArm(keyPos, keyVel, eefModel, Nu0, Q0));
        end
        
        if ~isempty(startCartFixed)
            graph.add(GaussianPriorWorkspacePositionArm(keyPos, eefModel, dof-1, Point3(startCartFixed), cartFixModel));
        end
        if ~isempty(startConfFixed)
            graph.add(PriorFactorVector(keyPos, startConfFixed, poseFixModel));
        end
    elseif i== totalTimeStep
        if ~isempty(goalCartFixed)
            graph.add(GaussianPriorWorkspacePositionArm(keyPos, eefModel, dof-1, Point3(goalCartFixed), cartFixModel));
        end
        if ~isempty(goalConfFixed)
            graph.add(PriorFactorVector(keyPos, goalConfFixed, poseFixModel));
        end
    end

    % GP priors
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, ...
            dt, QcModel));
        
        if isa(eefModel, 'PointRobotModel')
            graph.add(GaussianProcessPriorCartesianLinearPointRobot(key_pos1, key_vel1, key_pos2, key_vel2, ...
                eefModel, Phi{i}, Nu{i}, Q{i}));
        else
            graph.add(GaussianProcessPriorCartesianLinearArm(key_pos1, key_vel1, key_pos2, key_vel2, ...
                eefModel, Phi{i}, Nu{i}, Q{i}));
        end
        
        % cost factor
        
        if isa(armModel, 'PointRobotModel')
            graph.add(ObstaclePlanarSDFFactorPointRobot(...
                keyPos, armModel, obsSDF, obsSigma, obsEpsilon));
            graph_obs.add(ObstaclePlanarSDFFactorPointRobot(...
                keyPos, armModel, obsSDF, obsSigma, obsEpsilon));
        else
            graph.add(ObstacleSDFFactorArm(...
                keyPos, armModel, obsSDF, obsSigma, obsEpsilon));
            graph_obs.add(ObstacleSDFFactorArm(...
                keyPos, armModel, obsSDF, obsSigma, obsEpsilon));
        end
    end
end


parameters = LevenbergMarquardtParams;
parameters.setVerbosity('ERROR');
parameters.setlambdaInitial(1000.0);
parameters.setlambdaUpperBound(1e10);
parameters.setMaxIterations(maxIterations);
optimizer = LevenbergMarquardtOptimizer(graph, initValues, parameters);

fprintf('Initial Error = %d\n', graph.error(initValues))
fprintf('Initial Collision Cost: %d\n', graph_obs.error(initValues))

optimizer.optimize();
result = optimizer.values();
% result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', graph_obs.error(result))

%% create output trajectory

trajRep = struct();
trajRep.time = [];
trajRep.conf = [];
trajRep.pos = [];


for i=0:totalTimeStep
    conf = result.atVector(symbol('x',i));
    vel = result.atVector(symbol('v',i));
    
    robot_conf = conf;
    if negateRootJoint
        robot_conf(1) = -robot_conf(1);
    end

    trajRep.time = [trajRep.time; i*dt];
    trajRep.pos = [trajRep.pos; eefModel.sphereCentersMat(conf).'];
    trajRep.conf = [trajRep.conf; robot_conf'];
end

%% Plotting

if optSettings.plot
    h=figure; ax = axes(h);hold on; view(30,30)
    set(h, 'Position', [-1200, 100, 1100, 1200]);
    plotDataset3D(priorParams.dataset, ax, [150 150 150]./255, 0);
    hold on; axis equal; grid on;   
    plot3(trajRep.pos(:,1), trajRep.pos(:,2), trajRep.pos(:,3), 'r', 'linewidth', 2);
    
    if isa(eefModel, 'PointRobotModel')
        plotEvidenceMap2D(obsDataset.map, obsDataset.origin_x, obsDataset.origin_y, obsDataset.cell_size);
    else
        plotMap3D(obsDataset.corner_idx, [obsDataset.origin_x, obsDataset.origin_y, obsDataset.origin_z], obsDataset.cell_size);
    end
    
end

if optSettings.animate
    h=figure; ax = axes(h);  hold on; grid on; view(30,30)
    set(h, 'Position', [-1200, 100, 1100, 1200]);
    plotDataset3D(priorParams.dataset, h, [150 150 150]./255, 0);
    start_conf = result.atVector(symbol('x',0));
    hcp = plotRobotModel(armModel, start_conf);
    plotMap3D(obsDataset.corner_idx, [obsDataset.origin_x, obsDataset.origin_y, obsDataset.origin_z], obsDataset.cell_size);
    
    xlimit = get(ax,'xlim');
    ylimit = get(ax, 'ylim');
    zlimit = get(ax, 'zlim');
    
    xlimit = [xlimit(1) - (xlimit(2)-xlimit(1))/3, xlimit(2) + (xlimit(2)-xlimit(1))/3];
    ylimit = [ylimit(1) - (ylimit(2)-ylimit(1))/3, ylimit(2) + (ylimit(2)-ylimit(1))/3];
    zlimit = [zlimit(1) - (zlimit(2)-zlimit(1))/3, zlimit(2) + (zlimit(2)-zlimit(1))/3];
    
    axis([xlimit, ylimit, zlimit]);
    
    h2 = [];
    for i=1:totalTimeStep
        conf = result.atVector(symbol('x',i));
        
        delete(hcp); hcp = plotRobotModel(armModel,conf);
        delete(h2); h2 = plot3(trajRep.pos(1:i,1), trajRep.pos(1:i,2), trajRep.pos(1:i,3), 'r', 'linewidth', 2);
        pause(0.00001);
    end
end

end



