function Y=af7r61_decasteljau_batch_exact_original(C,u)
u=reshape(double(u),1,1,[]);
T=repmat(double(C),1,1,numel(u));
while size(T,2)>1
    T=(1-u).*T(:,1:end-1,:)+u.*T(:,2:end,:);
end
Y=reshape(T(:,1,:),3,[]);
end


function [z,tSym]=parse_symbolic_triplet(fstr)
s=strtrim(char(fstr));
tok=regexp(s,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
assert(~isempty(tok)&&numel(tok)==2);
varName=char(tok{1});
body=strtrim(char(tok{2}));
body=strrep(body,'.^','^');
body=strrep(body,'.*','*');
body=strrep(body,'./','/');
z=str2sym(body); z=z(:);
assert(numel(z)==3);
tSym=sym(varName,'real');
sv=symvar(z);
for k=1:numel(sv)
    assert(strcmp(char(sv(k)),varName));
end
end


function Z=normalize_triplet_output(z,N)
if N==1
    assert(numel(z)==3); Z=reshape(z,3,1); return;
end
if isequal(size(z),[3,N]), Z=z;
elseif isequal(size(z),[N,3]), Z=z.';
elseif numel(z)==3, Z=repmat(reshape(z,3,1),1,N);
else, error('[AF7R33] nominal vector handle output shape invalid.');
end
end


function v=normalize_scalar_output(v,N)
if isscalar(v), v=repmat(v,1,N);
elseif numel(v)==N, v=reshape(v,1,N);
else, error('[AF7R33] nominal scalar handle output shape invalid.');
end
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
