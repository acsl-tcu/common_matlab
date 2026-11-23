function data = Normalization(Data)
% Use the same state coordinates for current and next states.
assert(isequal(size(Data.X),size(Data.Y)), 'Normalization:StateSize', ...
    'X and Y must have identical state and sample dimensions.');
assert(size(Data.X,2)==size(Data.U,2), 'Normalization:SampleSize', ...
    'X, Y and U must have the same number of samples.');
assert(all(isfinite([Data.X(:);Data.Y(:);Data.U(:)])), ...
    'Normalization:Nonfinite','Training data must be finite.');
data.meanValue.x = mean(Data.X,2);
data.meanValue.y = data.meanValue.x;
data.meanValue.u = mean(Data.U,2);
data.stdValue.x = std(Data.X,0,2);
data.stdValue.u = std(Data.U,0,2);
data.stdValue.x(data.stdValue.x < 1e-12) = 1;
data.stdValue.u(data.stdValue.u < 1e-12) = 1;
data.stdValue.y = data.stdValue.x;
data.x = (Data.X-data.meanValue.x)./data.stdValue.x;
data.y = (Data.Y-data.meanValue.x)./data.stdValue.x;
data.u = (Data.U-data.meanValue.u)./data.stdValue.u;
end
