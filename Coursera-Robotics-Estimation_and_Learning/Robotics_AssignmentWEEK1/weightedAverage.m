function [val] = weightedAverage(weights, values)
%weightedAverage calculate the weighted average of values by applying the
%'weights'

%values - datapoints to average, one per row
%weights - weight to apply to each data point, one per row

%returns:
%val - the weighted average of 'values'

val = weights' * values;

val = val./ sum(weights, 1);
end