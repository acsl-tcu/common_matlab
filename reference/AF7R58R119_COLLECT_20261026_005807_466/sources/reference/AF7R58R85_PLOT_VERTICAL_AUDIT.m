function R=AF7R58R85_PLOT_VERTICAL_AUDIT()
% Read-only audit of graphics LINE data already present after SimHL.
% Distinguishes a same-time vertical plotting stroke from genuine physics.
% No invented velocity limit, no trajectory modification, no flight rerun.
figs=findall(groot,'Type','figure');
rows=struct('figure_number',{},'line_name',{},'same_time_pairs',{}, ...
    'max_same_time_value_gap',{},'large_gap_times',{});
for jf=1:numel(figs)
    L=findall(figs(jf),'Type','line');
    for k=1:numel(L)
        try
            t=double(get(L(k),'XData')); v=double(get(L(k),'YData'));
        catch,continue;end
        if numel(t)<2||numel(t)~=numel(v),continue;end
        t=t(:);v=v(:);
        good=isfinite(t)&isfinite(v);
        if nnz(good)<2,continue;end
        idx=find(good(1:end-1)&good(2:end));
        if isempty(idx),continue;end
        dT=abs(t(idx+1)-t(idx));
        localScale=max([ones(numel(idx),1),abs(t(idx)),abs(t(idx+1))],[],2);
        eqT=dT<=64*eps(localScale);
        dv=abs(v(idx+1)-v(idx));
        vs=max([ones(numel(idx),1),abs(v(idx)),abs(v(idx+1))],[],2);
        jump=eqT & dv>4096*eps(vs);
        if ~any(jump),continue;end
        nm=get(L(k),'DisplayName');
        if isempty(nm),nm='(unnamed)';end
        rows(end+1)=struct( ... %#ok<AGROW>
            'figure_number',double(get(figs(jf),'Number')), ...
            'line_name',char(nm), ...
            'same_time_pairs',nnz(jump), ...
            'max_same_time_value_gap',max(dv(jump)), ...
            'large_gap_times',t(idx(jump)).');
    end
end
R=struct('status','R85_DISPLAYED_LINE_GEOMETRY_ONLY', ...
    'same_time_vertical_lines',rows,'plot_evidence_is_not_hlc_data',true, ...
    'production_modified',false);
rootDir=fileparts(mfilename('fullpath'));
R.report_file=fullfile(rootDir,'AF7R58R85_PLOT_VERTICAL_REPORT.txt');
fid=fopen(R.report_file,'wt');assert(fid>0,'R85:ReportWriteFailed');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R85 plotted line geometry audit - NOT actual trajectory proof\n');
for j=1:numel(rows)
 a=rows(j);fprintf(fid,'fig=%g line=%s pairs=%d max_gap=%.9g times=%s\n', ...
     a.figure_number,a.line_name,a.same_time_pairs, ...
     a.max_same_time_value_gap,mat2str(a.large_gap_times,9));
end
fprintf(fid,'Zero findings does not rule out a plot artifact or physical discontinuity.\n');
end
