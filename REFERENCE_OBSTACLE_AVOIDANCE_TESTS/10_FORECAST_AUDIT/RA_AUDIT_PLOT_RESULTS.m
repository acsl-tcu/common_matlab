function RA_AUDIT_PLOT_RESULTS(A)
% Test-only visual summary: achieved empirical errors, never safety bounds.
if ~isstruct(A) || ~isfield(A,'flights') || isempty(A.flights)
 error('RAAudit:Plots','Run Flight audit first');
end
figure('Name','V8 4s empirical error vs flight truth','Color','w');
hold on;
lg={};
for k=1:numel(A.flights)
 f=A.flights(k);
 if ~isfield(f.stats,'p95Combined_m'),continue,end
 x=f.stats.horizons_s;y=f.stats.p95Combined_m;
 plot(x,y,'-o','LineWidth',1.3);
 lg{end+1}=f.name; %#ok<AGROW>
end
xlabel('Prediction horizon [s]');ylabel('p95 of max(drone,load) position error [m]');
title('Empirical post-hoc error; NOT a guaranteed safety tube');
grid on;legend(lg,'Location','northwest');hold off;
figure('Name','V8 worst-case prediction errors','Color','w');hold on;
lg={};
for k=1:numel(A.flights)
 f=A.flights(k);
 if ~isfield(f.stats,'maxCombined_m'),continue,end
 plot(f.stats.horizons_s,f.stats.maxCombined_m,'-s','LineWidth',1.3);
 lg{end+1}=f.name; %#ok<AGROW>
end
plot([0 4],[.1 .1],'k--','HandleVisibility','off');
plot([0 4],[.5 .5],'k:','HandleVisibility','off');
plot([0 4],[3 3],'k-.','HandleVisibility','off');
xlabel('Prediction horizon [s]');ylabel('Maximum observed error [m]');
title('All observed starts (0.1, 0.5, 3 m lines are comparisons, NOT bounds)');
grid on;legend(lg,'Location','northwest');hold off;
end
