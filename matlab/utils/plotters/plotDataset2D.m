function figHandle = plotDataset2D(dataset, figHandle)

if nargin ==1
    figHandle = figure;
end

set(0, 'currentfigure', figHandle);
hold on;

legendVec = cell(size(dataset));
for ind = 1:size(dataset,1)
    plot(dataset(ind).pos(:,1), dataset(ind).pos(:,2),'linewidth', 2);
    legendVec{ind} = sprintf('Demo %i', ind);
end

legend(legendVec);
xlabel('x_1'); ylabel('x_2');
grid on;box on;
axis equal;

end

