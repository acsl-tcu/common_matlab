#include "af7r36_core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace af7r34;
static void require_(bool ok, const char *why){ if(!ok)throw std::runtime_error(why); }
static bool close_(double a, double b){return std::abs(a-b) <= 2e-11*std::max({1.,std::abs(a),std::abs(b)});}
int main(){
    NominalExpr nom;
    std::vector<std::string> f(21,"0");
    f[0]="t"; f[1]="(2*t)"; f[2]="(0.3*t)";
    f[3]="1"; f[4]="2"; f[5]="0.3";
    nom.compile(f,"R21_LINEAR_BASE");
    ActiveReference ref;ref.nominal=&nom;ref.flight_t0=0;
    for(const auto&K:std::vector<std::vector<double>>{
            {0,1,2,3},{0,.5,1.5,4},{0,.75,1.5,3.,4.5},
            {0,.25,.5,.75,1.,1.25}}){
        const auto E=r58_build_normal_elimination(K,ref);
        require_(E.ok,"normal elimination failed");
        const auto H=r58_spatial_hessian3(K,2.0,9.81);
        const auto I=r58r14_lowjet_indices((int)K.size());
        const auto Hfull=r58_reduce_hessian(H,E);
        const auto old=r58r14_principal_hessian(Hfull,E.nred,I);
        const auto newer=r58_reduce_hessian_selected(H,E,I);
        require_(old.size()==newer.size(),"reduced hessian dimension mismatch");
        for(size_t i=0;i<old.size();++i)
            require_(old[i]==newer[i],"reduced Hessian binary64 value changed");
        std::vector<Row> rows(12);
        for(size_t r=0;r<rows.size();++r){
            rows[r].a.resize(E.nphys);
            for(size_t i=0;i<rows[r].a.size();++i)
                rows[r].a[i]=std::sin(0.22+(r+1)*(i+2)*.072);
            rows[r].b=.2+r*.05;
        }
        auto rrOld=rows,rrNew=rows;
        require_(r58_reduce_rows(rrOld,E),"legacy row reduction failed");
        require_(r58r14_project_rows(rrOld,E.nred,I),"legacy row projection failed");
        require_(r58_reduce_rows_selected(rrNew,E,I),"R21 selected row reduction failed");
        for(size_t r=0;r<rows.size();++r){
            require_(rrOld[r].a.size()==rrNew[r].a.size(),"row dimensions changed");
            for(size_t i=0;i<rrOld[r].a.size();++i)
                require_(rrOld[r].a[i]==rrNew[r].a[i],"row byte-level sum changed");
        }
        std::cout<<"EQUIVALENT K="<<K.size()<<" nphys="<<E.nphys
                 <<" nfull="<<E.nred<<" nlow="<<I.size()<<"\n";
    }
    // Replanning on an already-repaired reference is represented by additional
    // exact-C7 Correction layers, NOT by resetting the nominal clock.
    {std::array<Vec3,8> j0{},j1{};j1[0]={0,.2,0};
    Correction C;C.t0=1;C.t1=2;CorrSegment sg;
    sg.t0=1;sg.t1=2;sg.cp=vector_control_from_jets(j0,j1,1);
    C.seg.push_back(sg);ref.repairs.push_back(C);
    auto l=ref.eval(1),r=ref.eval(2.5);
    require_(close_(l[0].y,2.),"active-reference start changed");
    require_(close_(r[0].y,5.2),"active-reference terminal hold lost");
    for(int k=1;k<=6;++k){
      const auto at=ref.eval(2.);
      require_(close_(at[k].y,(k==1)?2.:0.),"active derivative continuity lost");
    }
    auto E=r58_build_normal_elimination({2.5,3.0,3.5,4.0},ref);
    require_(E.ok,"moving-normal elimination rejects already-repaired reference");
    std::cout<<"ACTIVE_REFERENCE_EVAL_AND_REPLAN_BASIS_PASS\n";
    }
    std::cout<<"R21_EQUIVALENCE_PASS\n";
}
