function [A, V] = updateParamsMLE(lambdaRidge, XTrain, YTrain, WTrain)
%UPDATEPARAMSMLE calculates discrete time-varying linear dynamics using
%maximum likelihood. estimation such that:
%               X_{t+1} = A_t*X_t + N_t,  where N_t ~ N(0, V_t)
% Weights can also be provided to bias the learned dynamics towards certain
% datapoints more.
%   
%   Input Format:
%   lambdaRidge         (scalar) regularization coefficient
%   XTrain              (cell array) X_t for all the demonstrations 
%   YTrain              (cell array) X_{t+1} for all the demonstrations
%   WTrain              (cell array) Weights for all the demonstrations
%
%
%   Output Format:
%   A                   (cell array) transition matrices
%   V                   (cell array) additive noise covariances


numPts = length(YTrain) + 1;
numDim = size(YTrain{1}, 3);
numDemos = size(YTrain{1},2);

A = cell(1, numPts-1);
V = cell(1, numPts-1);

for d=1:numDim
    for n=1:numPts-1
        Xn = XTrain{n}(:,:,d);
        Yn = YTrain{n}(:,:,d);
        
        if nargin==3
            Wn = eye(numDemos);
        else
            Wn = WTrain{n};
        end
        
        [An, Vn] = MLE(Xn, Yn, lambdaRidge, Wn);
        
        A{n}(:,:,d) = An;
        V{n}(:,:,d) = Vn;
        
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, V] = MLE(X, Y, lambda_ridge, W)

numStates = size(X,1)-1;

I_ridge = diag([0, ones(1, numStates)]);

A = (Y*W*X')/(X*W*X' + lambda_ridge*I_ridge);
        
z =  ((trace(W))^2 - trace(W'*W))/trace(W);
        
V = (Y - A*X)*W*(Y - A*X)'/z;

end
