function part=wp_item(kind,center,tag,radius,R,axes,p0,p1)
% Uniform schema, reused for C/B/D.
if nargin<5 || isempty(R), R=eye(3);end
if nargin<6 || isempty(axes), axes=zeros(3,1);end
if nargin<7 || isempty(p0), p0=center;end
if nargin<8 || isempty(p1), p1=center;end
part=struct('kind',char(kind),'center',center(:), ...
    'tag',char(tag),'radius',radius,'R',R,'axes',axes(:), ...
    'p0',p0(:),'p1',p1(:));
end
