function blocks=obs_budgeted_boxes(C,budget,targetSize)
% Conservative bounded-complexity decomposition for sphere/ellipsoid covers.
% 1) Cover each component with its AABB (or groups of adjacent AABBs if
%    there are more components than the allowed part count).
% 2) Optionally subdivide the largest long box, at most 'budget' boxes.
% The union of the final boxes contains the original obstacle. Each box
% will later be enclosed by ONE sphere/ellipsoid, so no part can be missed.
if nargin<3,targetSize=3.0;end
G=obs_budgeted_groups(C,budget);
blocks=struct('mid',{},'half',{},'componentIndex',{});
for i=1:numel(G)
    blocks(end+1)=struct('mid',(G(i).lo+G(i).hi)/2, ...
        'half',(G(i).hi-G(i).lo)/2, ...
        'componentIndex',G(i).indices(1)); %#ok<AGROW>
end
while numel(blocks)<budget && ~isempty(blocks)
    maxWidths=zeros(numel(blocks),1);
    axesToSplit=zeros(numel(blocks),1);
    for i=1:numel(blocks)
        [maxWidths(i),axesToSplit(i)]=max(2*blocks(i).half);
    end
    [biggest,idx]=max(maxWidths);
    if biggest<=targetSize+1e-10,break;end
    ax=axesToSplit(idx);
    old=blocks(idx);
    newHalf=old.half;newHalf(ax)=old.half(ax)/2;
    shift=zeros(1,3);shift(ax)=newHalf(ax);
    left=old;left.half=newHalf;left.mid=old.mid-shift;
    right=old;right.half=newHalf;right.mid=old.mid+shift;
    blocks(idx)=left;
    blocks(end+1)=right; %#ok<AGROW>
end
end
