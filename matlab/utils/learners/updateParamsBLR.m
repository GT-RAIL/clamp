function [MPosterior, KPosterior, SPosterior, NPosterior, VMAP] = updateParamsBLR(MPrior, KPrior, SPrior, NPrior, XTrain, YTrain, WTrain)

numPts = length(YTrain) + 1;
numDim = size(YTrain{1}, 3);
numDemos = size(YTrain{1},2);

MPosterior = cell(1, numPts-1);
KPosterior = cell(1, numPts-1);
SPosterior = cell(1, numPts-1);
NPosterior = cell(1, numPts-1);
VMAP = cell(1, numPts-1);

for d=1:numDim
    for n=1:numPts-1
        % Training data
        Xn = XTrain{n}(:,:,d); 
        Yn = YTrain{n}(:,:,d);
        
        if nargin==6
            Wn = eye(numDemos);
        else
            Wn = WTrain{n};
        end
        
        % BLR priors
        MnPrior = MPrior{n}(:,:,d);
        KnPrior = KPrior{n}(:,:,d);
        SnPrior = SPrior{n}(:,:,d);
        NnPrior = NPrior{n}(:,d);
        
        
        [MnPosterior, KnPosterior, SnPosterior, NnPosterior, VnMAP] = ...
            BLR(Xn, Yn, Wn, MnPrior, KnPrior, SnPrior, NnPrior);
                
        MPosterior{n}(:,:,d) = MnPosterior;
        KPosterior{n}(:,:,d) = KnPosterior;
        SPosterior{n}(:,:,d) = SnPosterior;
        NPosterior{n}(1,d) = NnPosterior;
        VMAP{n}(:,:,d) = VnMAP;
        
    end
end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [posteriorM, posteriorK, posteriorS, posteriorN, mapV] = BLR(X, Y, W, priorM, priorK, priorS, priorN)
% Model: Y = XB, where Y(i,:) = yi' and X(i,:) = xi' and B(:,i)= bi 
% ref: https://tminka.github.io/papers/minka-linear.pdf
% ref: https://en.wikipedia.org/wiki/Bayesian_multivariate_linear_regression


% Vn = V0 + (Y - X*posterior_B)'*(Y - X*posterior_B) + (posterior_B - prior_mu_B)'*prior_K*(posterior_B - prior_mu_B);
% vn = v0 + n;

% posterior_K = X'*W*X + prior_K;
% posterior_B = posterior_K\(X'*W*Y + prior_K*prior_B);
% posterior_V = prior_V;

m = size(X,1);
d = size(Y,1);
N = size(Y,2);


Sxx = X*W*X' + priorK;
Syx = Y*W*X' + priorM*priorK;
Syy = Y*W*Y' + priorM*priorK*priorM';
Sy_x = Syy - (Syx/Sxx)*Syx';

posteriorM = Syx/(Sxx);
posteriorK = Sxx;
posteriorS = Sy_x + priorS;
posteriorN = N + priorN;

%modeA = posteriorM;
mapV = posteriorS/(posteriorN + d + 1);

end
