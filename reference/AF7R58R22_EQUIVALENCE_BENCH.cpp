#include "af7r36_core.hpp"
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <cstring>
using namespace af7r34;
int main(){
    NominalExpr nom;
    std::vector<std::string> f(21,"0");
    f[0]="t"; f[1]="2*t"; f[2]="0.3*t";
    f[3]="1";f[4]="2";f[5]="0.3";
    nom.compile(f,"R22_LINEAR");
    ActiveReference ref;ref.nominal=&nom;
    double maxerr=0;int ntests=0;
    for(const auto& K:std::vector<std::vector<double>>{
      {0,1,2,3},{0,.5,1.5,4},{0,.75,1.5,3.,4.5},
      {0,.25,.5,.75,1.,1.25},{0,.4,1.1,2.2,3.0,5.0,7.0}}){
      const auto E=r58_build_normal_elimination(K,ref);
      if(!E.ok)throw std::runtime_error("normal elimination failed");
      const auto I=r58r14_lowjet_indices((int)K.size());
      for(const auto L:std::vector<double>{0.7,2.,3.7})
      for(const auto g:std::vector<double>{8.9,9.81,11.}){
         const auto h=r58_spatial_hessian3(K,L,g);
         const auto old=r58_reduce_hessian_selected(h,E,I);
         const auto newer=r58r22_lowjet_hessian_direct(K,L,g,E,I);
         if(old.size()!=newer.size()) throw std::runtime_error("size mismatch");
         for(size_t j=0;j<old.size();++j){
           const double error=std::abs(old[j]-newer[j]);
           maxerr=std::max(maxerr,error/std::max({1.,std::abs(old[j]),std::abs(newer[j])}));
           if(error>1e-12*std::max({1.,std::abs(old[j]),std::abs(newer[j])}))
             throw std::runtime_error("R22 Hessian equivalence failed");
         }
         ++ntests;
      }
    }
    const std::vector<double>K{0,.5,1.5,4};
    const auto E=r58_build_normal_elimination(K,ref);
    const auto I=r58r14_lowjet_indices((int)K.size());
    volatile double sink=0.0;
    const int N=200;
    auto t=std::chrono::steady_clock::now();
    for(int j=0;j<N;++j){
      const auto h=r58_spatial_hessian3(K,2.,9.81);
      const auto x=r58_reduce_hessian_selected(h,E,I);
      sink=sink+x[0];
    }
    auto t1=std::chrono::steady_clock::now();
    for(int j=0;j<N;++j){
      const auto x=r58r22_lowjet_hessian_direct(K,2.,9.81,E,I);
      sink=sink+x[0];
    }
    auto t2=std::chrono::steady_clock::now();
    std::cout<<"R22_EQUIVALENCE_PASS tests="<<ntests<<" max_rel="<<maxerr<<"\n";
    std::cout<<"R22_HESSIAN_BENCH old_us="<<std::chrono::duration<double,std::micro>(t1-t).count()/N
      <<" new_us="<<std::chrono::duration<double,std::micro>(t2-t1).count()/N<<" sink="<<sink<<"\n";
}
