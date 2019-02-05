function [figHandle] = plotGPTime(time, Mu, Sigma, numDim, numStates, numPts, color)
% Plots a Gaussian over time
% Inputs:
%       time: a vector of time (1 x N)
%       mu: mean vector (D x N) where N is the num_pts
%       Sigma: cell of covariances (D x D x N)
%       stateInd: the index of the selected state 
%       color: color of the mean in normalized RGB vector
%       figHandle: handle of the figure to be plotted on

N = 1.96;
figHandle = figure; hold on;

k = 1;
for ii =1:numStates
    if ii ==1
        fieldlabel = 'x';
    elseif ii==2
        fieldlabel = '\dot{x}';
    end
    
    shiftInd = numStates*numDim*(0:(numPts-1)) + numDim*(ii - 1);
    stateInd = repelem(shiftInd,numDim) + repmat(1:numDim,1,numPts);

    stateMu = Mu(stateInd,:);
    stateSigma = Sigma(stateInd,stateInd);
    
    for jj = 1:numDim
        h = subplot(numStates, numDim, k); hold on; grid on; box on; axis tight
        
        dimMu = stateMu(jj:numDim:end, :);
        dimSigma = stateSigma(jj:numDim:end, jj:numDim:end);
        
        plotGauss(time, dimMu, dimSigma, N, color, h);
        set( get(h,'XLabel'), 'String', '{\boldmath$t$}', 'Interpreter','latex','FontSize',20);
        ylabel =  ['{\boldmath$', fieldlabel, '_', int2str(jj),'$}'];
        set( get(h,'YLabel'), 'String', ylabel,'Interpreter','latex','FontSize',20);
        hold off;
        k = k+1; 
    end
end



end

%%

function axisHandle = plotGauss(x, yMu, ySigma, N, color, axisHandle)

if nargin <=5
    axisHandle = figure;
end

axes(axisHandle);

numPts = length(x);
lightcolor = color + [0.6,0.6,0.6];
lightcolor(find(lightcolor>1.0)) = 1.0;


ymax = zeros(1, numPts);
ymin = zeros(1, numPts);

for n=1:numPts
    ymax(n) = yMu(n) + sqrt(N^2.*ySigma(n, n));
    ymin(n) = yMu(n) - sqrt(N^2.*ySigma(n, n));
end
patch([x(1:end)', x(end:-1:1)'], [ymax(1:end) ymin(end:-1:1)], lightcolor, 'LineStyle', 'none', 'FaceAlpha', 0.6);
plot(x, yMu, 'lineWidth', 2.5, 'color', color);

end


