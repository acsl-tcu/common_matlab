function S=AF7R58R89_SOURCE_CHECK()
root=fileparts(mfilename('fullpath'));
t=fullfile(root,'AF7R58R81_ADAPTIVE_DELAYED_RETURN.m');
base='4c5b12210992ac3b8e31f343edaba837a22f6145138e6b6c4a7fc640f83d8c4a';
after='bd922558261e8369c9eaae94ff7281c38837afe530559cbdf0f3bec194f7c1e9';
h=AF7R58R89_SHA256_FILE(t);
assert(strcmp(h,base)||strcmp(h,after),'R89:UnexpectedSource');
S=struct('status','R89_REFERENCE_SOURCE_BOUND','current_sha256',h, ...
    'installed',strcmp(h,after),'baseline',strcmp(h,base), ...
    'flight_class_modified',false,'trigger_switched',false);
end
