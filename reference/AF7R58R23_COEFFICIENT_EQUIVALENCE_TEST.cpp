#include "af7r36_core.hpp"
#include <iostream>
#include <array>
#include <cstring>
#include <cmath>
#include <stdexcept>
using namespace af7r34;
static std::vector<double> old_scalar_control_from_jets(
    const std::array<double,8>&j0,const std::array<double,8>&j1,double T){
    std::array<double,8> ds{},de{};ds[0]=j0[0];de[0]=j1[0];
    double ff=1;
    for(int r=1;r<=7;++r){ff*=16-r;ds[r]=j0[r]*std::pow(T,r)/ff;de[r]=j1[r]*std::pow(T,r)/ff;}
    std::vector<double>P(16,0);
    auto choose=[](int n,int k)->double{
        if(k<0||k>n)return 0; double v=1;for(int i=1;i<=k;++i)v*=double(n-k+i)/double(i);return v;
    };
    for(int k=0;k<=7;++k){
        double v0=0,v1=0;
        for(int r=0;r<=k;++r){
            double bc=choose(k,r);v0+=bc*ds[r];v1+=((r&1)?-1.0:1.0)*bc*de[r];
        }
        P[k]=v0;P[15-k]=v1;
    }
    return P;
}
int main(){
    r58r23_binomial_table();r58r23_gram_table();
    for(int k=0;k<1024;++k){
        std::array<double,8> a{},b{};
        for(int r=0;r<8;++r){a[r]=std::sin((k+1)*(r+1)*.03);b[r]=std::cos((k+2)*(r+1)*.05);}
        const double duration=0.05+(k%71)*.103;
        auto x=old_scalar_control_from_jets(a,b,duration);
        auto y=scalar_control_from_jets(a,b,duration);
        if(x.size()!=y.size()||std::memcmp(x.data(),y.data(),x.size()*sizeof(double))!=0){
            std::cerr<<"BITWISE COEFFICIENT MISMATCH k="<<k<<"\n";return 1;
        }
    }
    for(int d=0;d<=3;++d)for(int a=0;a<=15-d;++a)for(int b=0;b<=15-d;++b){
        if(r58r23_gram_table()[d][16*a+b]!=gram(15-d,a,b))return 2;
    }
    std::cout<<"R23_COEFFICIENT_EQUIVALENCE_PASS 1024x16 exact bitwise"<<std::endl;
}
