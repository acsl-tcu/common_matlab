%正規化を行うための関数

function  data = Normalization(data)
% 正規化(平均:0,標準偏差:1)

%平均値の算出
%セグウェイ，ドローンどっちも：Data
%

for i = 1:size(data.x,1)
    meanValue.X(i,:) = mean(data.X(i,:));
    meanValue.Y(i,:) = mean(data.Y(i,:));
end

    meanValue.U(1,:) = mean(data.U(1,:));



%標準偏差の算出
for i = 1:size(data.X,1)
    stdValue.x(i,:) = std(data.X(i,:));
    stdValue.Y(i,:) = std(data.Y(i,:));
end

    stdValue.U(1,:) = std(data.U(1,:));

%データの正規化
for i = 1:size(data.X,1)
    data.x(i,:) = (data.X(i,:) - meanValue.x(i))/stdValue.x(i);
    data.Y(i,:) = (data.Y(i,:) - meanValue.Y(i))/stdValue.Y(i);
end

    data.U(1,:) = (data.U(1,:)-meanValue.U(1))/stdValue.U(1);


data.meanValue.x = meanValue.x;
data.meanValue.Y = meanValue.Y;
data.meanValue.U = meanValue.U;
data.stdValue.x = stdValue.x;
data.stdValue.Y = stdValue.Y;
data.stdValue.U = stdValue.U;

end