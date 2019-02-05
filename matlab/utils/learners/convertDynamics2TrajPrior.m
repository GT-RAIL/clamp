function [MuTraj, KTraj, PhiMat, vMat, QMat] = convertDynamics2TrajPrior(Phi, v, Q, v0, Q0)

numPts = length(Phi)+1;
numVars = size(Phi{1},1);

PhiMat = convertPhiCell2Mat(Phi);

vMat = [v0; reshape(cell2mat(v), (numPts-1)*numVars, 1)];

QTemp = [Q0, Q];
QMat = blkdiag(QTemp{:});

MuTraj = PhiMat*vMat;
KTraj = PhiMat*QMat*PhiMat';

end

function PhiMat = convertPhiCell2Mat(Phi)
% collapse cellarray of transition matrices to a lifted transition matrix

numPts = length(Phi) + 1;
numVars = size(Phi{1},1);

PhiMat = eye(numVars*numPts);

for n=1:numPts
    for j=(n-1):-1:1
        PhiSelected = Phi{j};
        
        PhiMat((numVars*(n-1)+1):(numVars*n),(numVars*(j-1)+1):(numVars*j)) = ...
            PhiMat((numVars*(n-1)+1):(numVars*n),(numVars*(j)+1):(numVars*(j+1)))*PhiSelected;
        
    end

end

end