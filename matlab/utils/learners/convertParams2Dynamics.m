function [Phi, v, Q] = convertParams2Dynamics(A, V)
% Input:
%   A: cell array of size (numPts-1) with 3D tensor per entry.
%
%

numPts = length(A) + 1;
Phi = cell(1, numPts-1);
v = cell(1, numPts-1);
Q = cell(1,numPts-1);

numPts = length(Phi) + 1;

for n=1:numPts-1
    [Phi{n}, v{n}, Q{n}] = extractDynamics(A{n}, V{n});
end

end



function [Phik, vk, Qk] = extractDynamics(Ak, Vk)
numDim = size(Ak,3);
numStates = size(Ak,1);

Qk = zeros(numStates*numDim, numStates*numDim);
Phik = zeros(numStates*numDim, numStates*numDim);
vk = zeros(numStates*numDim, 1);

for d = 1:numDim
    indSelected = [d, d+numDim];
    vk(indSelected,:) = Ak(:, 1, d);
    Phik(indSelected,indSelected) = Ak(:,2:end,d);
    Qk(indSelected,indSelected) = Vk(:,:,d) + 1e-12*eye(size(Vk(:,:,d)));
end

end
