function plotGPXYZ(Mu, Sigma, numDim, numStates, numPts, color, N)

% For N standard deviations spread of data, the radii of the eliipsoid will
% be given by N*SQRT(eigenvalues).    
if nargin == 6
    N=1.96;
end


shiftInd = numStates*numDim*(0:(numPts-1));
posInd = repelem(shiftInd,numDim) + repmat(1:numDim,1,numPts);

posMu = Mu(posInd,:);
posSigma = Sigma(posInd,posInd);


lightcolor = color + [0.6,0.6,0.6];
lightcolor(find(lightcolor>1.0)) = 1.0;

lightcolorMat = zeros(1,1,3);
lightcolorMat(1,1,:) = lightcolor;
lightcolorMat = repmat(lightcolorMat, [21,21,1]);

skipSize = 1;


for j=1:skipSize:numPts
    ind = numDim*(j-1)+1;
    Cov = posSigma(ind:(ind+2), ind:(ind+2));
    mu = posMu(ind:(ind+2));
    
    [U,L] = eig(Cov);
    % L: eigenvalue diagonal matrix
    % U: eigen vector matrix, each column is an eigenvector
    
    radii = N*max(1e-2,sqrt(diag(L)));
    
    % generate data for "unrotated" ellipsoid
    [xc,yc,zc] = ellipsoid(0,0,0,radii(1),radii(2),radii(3));
    
    % rotate data with orientation matrix U and center mu
    a = kron(U(:,1),xc);
    b = kron(U(:,2),yc);
    c = kron(U(:,3),zc);
    
    data = a+b+c; n = size(data,2);
    
    x = data(1:n,:)+mu(1);
    y = data(n+1:2*n,:)+mu(2);
    z = data(2*n+1:end,:)+mu(3);
    
    C = (xc.^2 + yc.^2 + zc.^2);
    C = 1.0 - C./max(max(C)).*1.0;
    
    w = [.9 .9 .9];    %# middle    
    %# colormap of size 64-by-3, ranging from red -> white -> blue
    c = zeros(32,3);
    for i=1:3
        c(:,i) = linspace(w(i), color(i), 32);
    end
    
    

    % now plot the rotated ellipse
    sc = surf(x,y,z,lightcolorMat); shading interp;% colormap winter; %colormap(c);
    sc.EdgeColor = 'none';
    alpha(0.2)
    %alpha(0)
end

l = plot3(posMu(1:numDim:end), posMu(2:numDim:end), posMu(3:numDim:end),'Marker','.', 'lineWidth', 6, 'color', color);
uistack(l,'top');


end