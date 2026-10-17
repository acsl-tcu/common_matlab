function [C,A] = af7r57_common_dyadic_refinement(L,U)
% AF7R57_COMMON_DYADIC_REFINEMENT
% -----------------------------------------------------------------------------
% Exact common refinement of two leaf partitions.
%
% Input canonical form:
%   N x 3 double matrix [segment, u0, u1]
%
% Requirements:
%   - segment is a positive integer;
%   - 0 <= u0 < u1 <= 1;
%   - for each segment present in either partition, each input partition
%     covers [0,1] exactly with no interior overlap;
%   - dyadic endpoints are expected and are compared by exact IEEE-754
%     equality.  No tolerance, snapping, merging, or sampled acceptance.
%
% Output:
%   C : exact common refinement [segment,u0,u1]
%   A : formal audit struct.
% -----------------------------------------------------------------------------

L = af7r57_leaf_matrix(L,'lower');
U = af7r57_leaf_matrix(U,'upper');

af7r57_validate_partition(L,'lower');
af7r57_validate_partition(U,'upper');

segs = unique([L(:,1);U(:,1)]);
C = zeros(0,3);

for is = 1:numel(segs)
    s = segs(is);
    l = L(L(:,1)==s,2:3);
    u = U(U(:,1)==s,2:3);

    if isempty(l) || isempty(u)
        error('AF7R57:SegmentSetMismatch', ...
            'Segment %g is absent from one input partition.',s);
    end

    % The overlay/common refinement of two exact 1-D partitions is exactly
    % the sorted union of their boundary sets.
    B = unique([l(:);u(:)]);
    B = sort(B);

    if isempty(B) || B(1) ~= 0 || B(end) ~= 1
        error('AF7R57:NonUnitCover', ...
            'Segment %g does not have exact [0,1] boundary cover.',s);
    end

    for k = 1:numel(B)-1
        if ~(B(k) < B(k+1))
            error('AF7R57:DegenerateCommonLeaf', ...
                'Degenerate common leaf on segment %g.',s);
        end
        C(end+1,:) = [s,B(k),B(k+1)]; %#ok<AGROW>
    end
end

lowerRefined = af7r57_is_refinement(C,L);
upperRefined = af7r57_is_refinement(C,U);
exactCover   = af7r57_exact_cover(C);

A = struct();
A.exact_cover = exactCover;
A.lower_refined_exactly = lowerRefined;
A.upper_refined_exactly = upperRefined;
A.lower_leaf_count = size(L,1);
A.upper_leaf_count = size(U,1);
A.common_leaf_count = size(C,1);
A.tolerance_used = false;
A.sampled_acceptance = false;
A.closed = exactCover && lowerRefined && upperRefined;

if ~A.closed
    error('AF7R57:CommonRefinementNotClosed', ...
        'Exact common-refinement proof did not close.');
end
end


function X = af7r57_leaf_matrix(X,name)
if isnumeric(X)
    if ~(ismatrix(X) && size(X,2)==3)
        error('AF7R57:LeafShape','%s leaves must be N x 3 [segment,u0,u1].',name);
    end
    X = double(X);
    return;
end

if istable(X)
    vars = X.Properties.VariableNames;
    s = af7r57_pick_field(vars, {'segment','seg','span','piece'});
    a = af7r57_pick_field(vars, {'u0','ulo','u_lo','lower','lo'});
    b = af7r57_pick_field(vars, {'u1','uhi','u_hi','upper','hi'});
    X = [double(X.(s)),double(X.(a)),double(X.(b))];
    return;
end

if isstruct(X)
    f = fieldnames(X);
    s = af7r57_pick_field(f, {'segment','seg','span','piece'});
    a = af7r57_pick_field(f, {'u0','ulo','u_lo','lower','lo'});
    b = af7r57_pick_field(f, {'u1','uhi','u_hi','upper','hi'});
    X = [[X.(s)].',[X.(a)].',[X.(b)].'];
    X = double(X);
    return;
end

error('AF7R57:LeafType','Unsupported %s leaf container.',name);
end


function f = af7r57_pick_field(names, candidates)
f = '';
for k = 1:numel(candidates)
    j = find(strcmpi(names,candidates{k}),1);
    if ~isempty(j)
        f = names{j};
        return;
    end
end
error('AF7R57:LeafSchema', ...
    'Leaf schema is not recognized. Required semantics: segment,u0,u1.');
end


function af7r57_validate_partition(X,name)
if isempty(X)
    error('AF7R57:EmptyPartition','%s partition is empty.',name);
end
if any(~isfinite(X(:)))
    error('AF7R57:NonfiniteLeaf','%s partition contains nonfinite values.',name);
end
if any(X(:,1) ~= fix(X(:,1))) || any(X(:,1) < 1)
    error('AF7R57:SegmentIndex','%s segment indices must be positive integers.',name);
end
if any(X(:,2) < 0) || any(X(:,3) > 1) || any(X(:,2) >= X(:,3))
    error('AF7R57:LeafBounds','%s leaves violate 0<=u0<u1<=1.',name);
end

segs = unique(X(:,1));
for is = 1:numel(segs)
    s = segs(is);
    P = sortrows(X(X(:,1)==s,2:3),1);
    if P(1,1) ~= 0 || P(end,2) ~= 1
        error('AF7R57:CoverEndpoint', ...
            '%s segment %g does not exactly start at 0 and end at 1.',name,s);
    end
    if size(P,1) > 1 && any(P(1:end-1,2) ~= P(2:end,1))
        error('AF7R57:CoverGapOverlap', ...
            '%s segment %g has a gap or interior overlap.',name,s);
    end
end
end


function tf = af7r57_is_refinement(C,P)
tf = true;
for k = 1:size(C,1)
    s = C(k,1); a = C(k,2); b = C(k,3);
    Q = P(P(:,1)==s,:);
    ok = any(Q(:,2) <= a & b <= Q(:,3));
    if ~ok
        tf = false;
        return;
    end
end

% Every original leaf must be exactly reconstructed by consecutive common leaves.
for k = 1:size(P,1)
    s = P(k,1); a = P(k,2); b = P(k,3);
    Q = sortrows(C(C(:,1)==s & C(:,2)>=a & C(:,3)<=b,2:3),1);
    if isempty(Q) || Q(1,1) ~= a || Q(end,2) ~= b
        tf = false;
        return;
    end
    if size(Q,1)>1 && any(Q(1:end-1,2) ~= Q(2:end,1))
        tf = false;
        return;
    end
end
end


function tf = af7r57_exact_cover(C)
tf = true;
segs = unique(C(:,1));
for is = 1:numel(segs)
    s = segs(is);
    P = sortrows(C(C(:,1)==s,2:3),1);
    if isempty(P) || P(1,1) ~= 0 || P(end,2) ~= 1
        tf = false; return;
    end
    if size(P,1)>1 && any(P(1:end-1,2) ~= P(2:end,1))
        tf = false; return;
    end
end
end
