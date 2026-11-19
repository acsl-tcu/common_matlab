function info = check_CBF_REAL_LOAD_BETA4_SYMBOLIC(outputFolder)
% CHECK_CBF_REAL_LOAD_BETA4_SYMBOLIC
% Independent symbolic certificate that the load sphere / obstacle-ellipsoid
% separation function generically contains physical torques in derivative 4.
% Symbolic Math Toolbox required. This does NOT establish a global r=4 CBF.
if nargin<1,outputFolder=fileparts(mfilename('fullpath'));end
syms q0 q1 q2 q3 t1 t2 t3 p1 p2 p3 c1 c2 c3 real
syms M11 M12 M13 M22 M23 M33 T m mL Jx Jy Jz real
q=[q0;q1;q2;q3]; %#ok<NASGU>
pT=[t1;t2;t3];
pL=[p1;p2;p3]; c=[c1;c2;c3];
M=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
R=[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2); ...
   2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1); ...
   2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1*q1+q2*q2)];
D=[-R(:,2)/Jx R(:,1)/Jy sym(zeros(3,1))];
beta4=(2*T/(m+mL))*(pT.'*M*(pL-c))*(D.'*pT);
info=struct();
info.beta4=beta4;
info.isIdenticallyZero=[isequal(simplify(beta4(1)),sym(0)), ...
                        isequal(simplify(beta4(2)),sym(0)), ...
                        isequal(simplify(beta4(3)),sym(0))];
info.hasGenericOrder4Action=~all(info.isIdenticallyZero);
name=fullfile(outputFolder,'LOAD_BETA4_SYMBOLIC_PROOF.txt');
fid=fopen(name,'w');
if fid<0,error('CBFRealZOH:SymbolicReport','Cannot write proof');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'Closed-form load ellipsoid barrier h=(pL-c)'' M (pL-c)-1\n');
fprintf(fid,'M is a positive-definite constant; obstacle held fixed.\n');
fprintf(fid,'Coefficient of tau in h^(4), evaluated along the original physical model:\n');
fprintf(fid,' beta4 = 2*T/(m+mL) * (pT''*M*(pL-c)) * D''*pT\n');
fprintf(fid,' D=[-R(:,2)/Jx, R(:,1)/Jy, zeros(3,1)]\n\n');
for i=1:3
    fprintf(fid,'beta4_%d identically zero? %d\n%s\n',...
        i,info.isIdenticallyZero(i),char(beta4(i)));
end
if info.hasGenericOrder4Action
    fprintf(fid,'\nCONCLUSION: GENERIC LOAD TORQUE ORDER 4; REQUESTED 6 FAILS.\n');
else
    fprintf(fid,'\nCONCLUSION: formula evaluation requires review.\n');
end
end
