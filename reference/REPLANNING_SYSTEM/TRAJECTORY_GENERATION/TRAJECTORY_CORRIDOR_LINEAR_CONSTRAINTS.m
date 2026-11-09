function K=TRAJECTORY_CORRIDOR_LINEAR_CONSTRAINTS(C,max_rows)
%TRAJECTORY_CORRIDOR_LINEAR_CONSTRAINTS  Convert certified planes to A*x<=b.
% x=[vec(U_1);vec(L_1);vec(U_2);vec(L_2);...] with MATLAB column-major
% 3xN original Bernstein controls. For each time-leaf its de Casteljau
% map is linear, so constraints cover ALL continuous times of that leaf.
% Cable planes are applied to BOTH end-point polygons (not their centres).
% max_rows: caller-owned computational budget, NEVER a physical bound.
assert(isstruct(C)&&isscalar(C)&&isfield(C,'certified')&&C.certified && ...
    isfield(C,'leaves')&&isfield(C,'segment_info')&& ...
    isfield(C,'variable_count'), ...
    'TRAJECTORY_CORRIDOR_LINEAR:Uncertified','No certified corridor.');
assert(isnumeric(max_rows)&&isscalar(max_rows)&&isreal(max_rows)&& ...
    isfinite(max_rows)&&max_rows>=0&&max_rows==fix(max_rows), ...
    'TRAJECTORY_CORRIDOR_LINEAR:Budget','External row budget required.');
row_count=0;
for i=1:numel(C.leaves)
    L=C.leaves(i);
    nU=size(L.uav_map,2);nL=size(L.payload_map,2);
    row_count=row_count+nU*(size(L.uav_normals,2)+size(L.cable_normals,2))+ ...
        nL*(size(L.payload_normals,2)+size(L.cable_normals,2));
end
assert(row_count<=max_rows, ...
    'TRAJECTORY_CORRIDOR_LINEAR:Budget', ...
    'Linear inequality row count exceeds the caller budget.');
% Sparse triplets are generated from one normal and a Bezier subdivision
% column at a time; no dense matrix proportional to all constraints.
I=cell(1,row_count);J=cell(1,row_count);V=cell(1,row_count);
b=zeros(row_count,1);row=0;
for i=1:numel(C.leaves)
    leaf=C.leaves(i);s=leaf.segment_index;info=C.segment_info(s);
    [I,J,V,b,row]=append_planes(I,J,V,b,row,leaf.uav_map, ...
        leaf.uav_normals,leaf.uav_offsets,info.uav_variable_start);
    [I,J,V,b,row]=append_planes(I,J,V,b,row,leaf.payload_map, ...
        leaf.payload_normals,leaf.payload_offsets,info.payload_variable_start);
    [I,J,V,b,row]=append_planes(I,J,V,b,row,leaf.uav_map, ...
        leaf.cable_normals,leaf.cable_offsets,info.uav_variable_start);
    [I,J,V,b,row]=append_planes(I,J,V,b,row,leaf.payload_map, ...
        leaf.cable_normals,leaf.cable_offsets,info.payload_variable_start);
end
assert(row==row_count,'TRAJECTORY_CORRIDOR_LINEAR:Rows','Row-count mismatch.');
if row_count==0
    A=sparse(0,C.variable_count);
else
    A=sparse([I{:}],[J{:}],[V{:}],row_count,C.variable_count);
end
K=struct('Aineq',A,'bineq',b,'variable_count',C.variable_count, ...
    'rows',row_count,'status','CERTIFIED_CORRIDOR_LINEAR_INEQUALITIES', ...
    'safe_to_commit',false, ...
    'variable_order','[vec(U1);vec(L1);vec(U2);vec(L2);...]');
end
function [I,J,V,b,row]=append_planes(I,J,V,b,row,M,N,offsets,base)
assert(size(N,2)==numel(offsets), ...
    'TRAJECTORY_CORRIDOR_LINEAR:Plane','Normal/offset mismatch.');
for j=1:size(N,2)
    n=N(:,j);
    for k=1:size(M,2)
        coeff=reshape(n*M(:,k).',1,[]);
        row=row+1;
        nz=find(coeff~=0);
        I{row}=repmat(row,1,numel(nz));
        J{row}=base-1+nz;
        V{row}=-coeff(nz);
        b(row)=-offsets(j);
    end
end
end
