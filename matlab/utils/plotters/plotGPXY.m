function plotGPXY(Mu, Sigma, numDim, numStates, numPts, color)

shiftInd = numStates*numDim*(0:(numPts-1));
posInd = repelem(shiftInd,numDim) + repmat(1:numDim,1,numPts);

posMu = Mu(posInd,:);
posSigma = Sigma(posInd,posInd);

N = 1.96; % number of std deviations around mean to plot

%subplot(numStates,1,i); hold on; grid on; box on;
lightcolor = color + [0.6,0.6,0.6];
lightcolor(find(lightcolor>1.0)) = 1.0;

xmin = zeros(1, numPts);
xmax = zeros(1, numPts);

ymax = zeros(1, numPts);
ymin = zeros(1, numPts);

for j=1:numPts
    
    mu = posMu((numDim*(j-1)+1):numDim*j);
    S = posSigma((numDim*(j-1)+1):numDim*j, (numDim*(j-1)+1):numDim*j);
    
    [U,L] = eig(S);
    % L: eigenvalue diagonal matrix
    % U: eigen vector matrix, each column is an eigenvector
    
    % For N standard deviations spread of data, the radii of the eliipsoid will
    % be given by N*SQRT(eigenvalues).
    
    N = 1.96; % choose your own N
    radii = N*sqrt(diag(L));
    
    %     % generate data for "unrotated" ellipsoid
    [xc, yc] = meshgrid(linspace(-radii(1),radii(1),2), linspace(-radii(2),radii(2),2));
    
    % rotate data with orientation matrix U and center mu
    a = kron(U(:,1),xc);
    b = kron(U(:,2),yc);
    
    data = a+b; n = size(data,2);
    
    x = data(1:n,:)+mu(1);
    y = data(n+1:2*n,:)+mu(2);
    
    xmin(j) = x(1,2);
    ymin(j) = y(1,2);
    
    xmax(j) = x(2,1);
    ymax(j) = y(2,1);

end

xx = [xmin(1:end), xmax(end:-1:1)];
yy = [ymin(1:end), ymax(end:-1:1)];
patch(xx, yy, lightcolor, 'LineStyle', 'none', 'FaceAlpha', 0.6);
plot(posMu(1:numDim:end)', posMu(numDim:numDim:end)', 'Marker','.', 'lineWidth', 3.0, 'color', color);

% inspired from https://kittipatkampa.wordpress.com/2011/08/04/plot-3d-ellipsoid/

end

