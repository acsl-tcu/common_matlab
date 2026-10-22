#include "af7r36_core.hpp"
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace af7r34;

static void require(bool x,const char*m){if(!x)throw std::runtime_error(m);}
int main(){
    // An actual standalone C++17 regression test; no MATLAB dependencies.
    // All base-reference derivatives are exact symbolic literals.
    NominalExpr nom;
    std::vector<std::string> f(21,"0");
    f[0]="t";f[3]="1";
    nom.compile(f,"R14_TEST_STRAIGHT");
    ActiveReference ref;ref.nominal=&nom;ref.flight_t0=0;
    for(const auto&K:std::vector<std::vector<double>>{
      {0,1,2,3},{0,0.5,1.5,4},{0,0.75,2.25,4.5}}){
        auto E=r58_build_normal_elimination(K,ref);
        require(E.ok,"normal elimination failed");
        const int N=(int)K.size();
        auto I=r58r14_lowjet_indices(N);
        require((int)I.size()==4*(N-2)+3,"R14 dimension incorrect");
        require((int)I.size()<E.nred,"R14 not reduced");
        auto H=r58_spatial_hessian3(K,2.0,9.81);
        auto Hr=r58_reduce_hessian(H,E);
        auto Hs=r58r14_principal_hessian(Hr,E.nred,I);
        require((int)Hs.size()==(int)I.size()*(int)I.size(),"R14 hessian projection failed");
        std::vector<double>Hnorm, scales;
        double a,b,c,d;
        require(r58_equilibrate_hessian(Hs,(int)I.size(),Hnorm,scales,a,b,c,d),"R14 hessian balance failed");
        std::vector<double>cholL;
        require(chol(Hnorm,(int)I.size(),cholL),"R14 restricted Hessian not positive definite");
        // Verify energy identity for every basis direction and nestedness.
        for(int i=0;i<(int)I.size();++i){
            QPSol q;q.ok=true;q.z.assign(I.size(),0.0);q.z[i]=1;
            auto full=r58r14_lift_solution(q,E.nred,I);
            require(full.ok,"failed to lift");
            auto phys=r58_expand_solution(full,E);
            require(phys.ok,"failed to expand");
            double qpCost=Hs[i*I.size()+i],fullCost=0;
            for(int row=0;row<E.nred;++row)
                for(int col=0;col<E.nred;++col)
                    fullCost+=full.z[row]*Hr[row*E.nred+col]*full.z[col];
            require(std::fabs(fullCost-qpCost)<=1e-10*std::max(1.0,std::fabs(qpCost)),"energy mismatch");
        }
        // Solve the ACTUAL constrained QP on the nested low-jet subspace.
        // The constraints are on physical correction positions (not samples
        // of obstacle safety): exact QP feasibility is checked independently.
        std::vector<Row> rows;
        for(int node=1;node<N-1;++node){
            Row row;
            row.a.assign(E.nphys,0.0);
            const int qp=r58_scalar_index(node,0,N);
            row.a[3*qp+1]=1.0;
            row.b=(node==1)?0.1:0.12;
            rows.push_back(row);
        }
        require(r58_reduce_rows(rows,E),"normal row reduction failed");
        require(r58r14_project_rows(rows,E.nred,I),"nested row reduction failed");
        require(r58_apply_variable_scaling_to_rows(rows,scales),"row Hessian equilibration failed");
        double rmin=0,rmax=0;
        require(r58_equilibrate_rows(rows,rmin,rmax),"row equilibration failed");
        R58CGDiag cg;
        auto qtest=r58_solve_qp_constraint_generation(Hnorm,(int)I.size(),rows,cg);
        require(qtest.ok,"actual nested QP did not converge");
        require(cg.fullOptimalityClosed,"all-row KKT optimality not closed");
        for(const auto&r:rows){
            double az=0;
            for(size_t i=0;i<r.a.size();++i)az+=r.a[i]*qtest.z[i];
            require(az>=r.b-1e-10,"nested QP constraint violated");
        }
        auto testUnscaled=r58_unscale_solution(qtest,scales);
        auto testLifted=r58r14_lift_solution(testUnscaled,E.nred,I);
        auto testPhysical=r58_expand_solution(testLifted,E);
        require(testPhysical.ok,"nested feasible solution expansion failed");
        double ce=0,pe2=0,jm=0;std::array<double,3>rawUB{};
        auto feasiblePoly=r58_build_candidate(testPhysical,K,0.25,2.,9.81,{},ce,pe2,jm,rawUB);
        require(feasiblePoly.exact_c7_constructive,"nested feasible QP lost C7");
        std::cout<<"QP_PASS lowDof="<<I.size()<<" KKT="<<cg.fullOptimalityClosed
                 <<" iterations="<<cg.outerIterations
                 <<" velUB="<<feasiblePoly.dub[0]<<"\n";
        // An artificial shape in R14 subspace, never accepted from samples.
        QPSol low;low.ok=true;low.z.assign(I.size(),0.0);
        low.z[0]=0.1;low.z[4]=0.12;low.z.back()=0.1;
        auto full=r58r14_lift_solution(low,E.nred,I);
        auto phys=r58_expand_solution(full,E);
        double qe=0,pe=0,join=0;std::array<double,3>ubr{};
        auto C=r58_build_candidate(phys,K,0.25,2.,9.81,{},qe,pe,join,ubr);
        require(C.exact_c7&&C.exact_c7_constructive,"not exact C7");
        require(C.cp.size()==K.size()-1,"invalid correction segments");
        require(C.terminal_hold,"no terminal hold");
        require(std::isfinite(C.dub[0])&&std::isfinite(C.dub[1])&&std::isfinite(C.dub[2]),"nonfinite derivative bounds");
        for(int r=1;r<=7;++r){
            auto last=r31_endpoint_derivative(C.cp.back(),K.back()-K[K.size()-2],r,true);
            require(norm(last)<=1e-10,"not zero-jet terminal");
        }
        std::cout<<std::setprecision(12)<<"PASS K="<<K.size()
                 <<" normalDof="<<E.nred<<" lowDof="<<I.size()
                 <<" velUB="<<C.dub[0]
                 <<" accUB="<<C.dub[1]
                 <<" jerkUB="<<C.dub[2]
                 <<" maxScaledJetError="<<qe<<"\n";
    }
    // Non-affine (curved) moving normal: verify algebraic P/V coupling.
    NominalExpr curved;
    std::vector<std::string> fc(21,"0");
    fc[0]="t";fc[1]="0.2*t^2";
    fc[3]="1";fc[4]="0.4*t";fc[7]="0.4";
    curved.compile(fc,"R14_TEST_CURVED");
    ActiveReference refC;refC.nominal=&curved;refC.flight_t0=0;
    const std::vector<double> Kc={0,0.5,1.5,4};
    auto Ec=r58_build_normal_elimination(Kc,refC);
    require(Ec.ok,"curved normal elimination failed");
    auto Ic=r58r14_lowjet_indices((int)Kc.size());
    for(int node=1;node<(int)Kc.size()-1;++node){
        const auto x=refC.eval(Kc[node]);
        Vec3 v=x[1],a=x[2];double nv=norm(v);Vec3 e=v/nv;
        Vec3 ed=(a-e*dot(e,a))/nv;
        const int p=r58_scalar_index(node,0,(int)Kc.size());
        const int vv=r58_scalar_index(node,1,(int)Kc.size());
        const double lambda=r58_node_scale(Kc,node);
        for(int col:Ic){
            Vec3 P{Ec.B[(3*p)*Ec.nred+col],Ec.B[(3*p+1)*Ec.nred+col],Ec.B[(3*p+2)*Ec.nred+col]};
            Vec3 V{Ec.B[(3*vv)*Ec.nred+col],Ec.B[(3*vv+1)*Ec.nred+col],Ec.B[(3*vv+2)*Ec.nred+col]};
            require(std::fabs(dot(e,P))<1e-12,"curved normal position constraint violated");
            require(std::fabs(dot(ed,P)+dot(e,V)/lambda)<1e-12,"curved normal velocity constraint violated");
        }
    }
    std::cout<<"PASS CURVED MOVING-NORMAL EQUALITY\n";
    return 0;
}
