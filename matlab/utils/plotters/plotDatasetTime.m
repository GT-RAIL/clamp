function figHandle = plotDatasetTime(dataset)
% Plots Demonstrations against time


figHandle = figure; hold on;
allFields =  {'pos', 'vel', 'acc'};
fieldsFlag = isfield(dataset, allFields);
fields = allFields(fieldsFlag);
numFields = length(fields);

if numFields == 0
    error('The dataset is empty!');
end

numDim = size(eval(['dataset(1).', fields{1}]),2);
numDemos = size(dataset,1);

k = 1;
for i = 1:numFields
    fieldname = fields{i};
    if strcmp(fieldname,'pos')
        fieldlabel = 'x';
    elseif strcmp(fieldname,'vel')
        fieldlabel = '\dot{x}';
    elseif strcmp(fieldname,'acc')
        fieldlabel = '\ddot{x}';
    end
    
    for j = 1:numDim
        h = subplot(numFields, numDim, k); hold on; axis tight; grid on; box on;
        legendVec = cell(size(dataset));
        for l = 1:numDemos
            fieldData = eval(['dataset(l).', fieldname]);
            plot(dataset(l).time, fieldData(:,j), 'linewidth', 2);
            legendVec{l} = sprintf('Demo %i', l);
        end
        legend(legendVec); 
        set( get(h,'XLabel'), 'String', '{\boldmath$t$}', 'Interpreter','latex','FontSize',20);
        ylabel =  ['{\boldmath$', fieldlabel, '_', int2str(j),'$}'];
        set( get(h,'YLabel'), 'String', ylabel,'Interpreter','latex','FontSize',20);
        hold off;
        k = k + 1;
    end    
end


end
