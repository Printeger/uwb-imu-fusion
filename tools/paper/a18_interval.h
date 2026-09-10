#pragma once
// Existing MPFR 4 ABI subset, no dependency installation. Layout verified at runtime.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <vector>
extern "C" {
struct A18Mpfr { long precision; int sign; long exponent; unsigned long* limbs; };
void mpfr_init2(A18Mpfr*,long); void mpfr_clear(A18Mpfr*);
int mpfr_set_d(A18Mpfr*,double,int); int mpfr_set(A18Mpfr*,const A18Mpfr*,int);
int mpfr_set_str(A18Mpfr*,const char*,int,int);
int mpfr_add(A18Mpfr*,const A18Mpfr*,const A18Mpfr*,int);
int mpfr_sub(A18Mpfr*,const A18Mpfr*,const A18Mpfr*,int);
int mpfr_mul(A18Mpfr*,const A18Mpfr*,const A18Mpfr*,int);
int mpfr_div(A18Mpfr*,const A18Mpfr*,const A18Mpfr*,int);
int mpfr_neg(A18Mpfr*,const A18Mpfr*,int);
int mpfr_sqrt(A18Mpfr*,const A18Mpfr*,int);
int mpfr_sin(A18Mpfr*,const A18Mpfr*,int); int mpfr_cos(A18Mpfr*,const A18Mpfr*,int);
int mpfr_tan(A18Mpfr*,const A18Mpfr*,int); int mpfr_acos(A18Mpfr*,const A18Mpfr*,int);
int mpfr_cmp(const A18Mpfr*,const A18Mpfr*); int mpfr_number_p(const A18Mpfr*);
double mpfr_get_d(const A18Mpfr*,int);
char* mpfr_get_str(char*,long*,int,size_t,const A18Mpfr*,int);void mpfr_free_str(char*);
const char* mpfr_get_version();
}
namespace a18 {
constexpr int DOWN=3,UP=2,NEAR=0;constexpr long PREC=333;
struct Failure:std::runtime_error {std::string status;Failure(std::string s,std::string why):std::runtime_error(s+":"+why),status(std::move(s)){} };
[[noreturn]] inline void unsupported(const std::string&s){throw Failure("NUMERIC_REFERENCE_UNSUPPORTED",s);}
[[noreturn]] inline void nonfinite(const std::string&s){throw Failure("NUMERIC_REFERENCE_NONFINITE",s);}
struct M {
 A18Mpfr x;
 M(double d=0){mpfr_init2(&x,PREC);mpfr_set_d(&x,d,NEAR);}
 M(const M&a){mpfr_init2(&x,PREC);mpfr_set(&x,&a.x,NEAR);}
 M(M&&a)noexcept:x(a.x){mpfr_init2(&a.x,PREC);mpfr_set_d(&a.x,0,NEAR);}
 M& operator=(const M&a){mpfr_set(&x,&a.x,NEAR);return *this;}
 ~M(){mpfr_clear(&x);}
 bool finite()const{return mpfr_number_p(&x);}
 double d(int rnd=NEAR)const{return mpfr_get_d(&x,rnd);}
 std::string binary()const{if(!finite())return "nonfinite";long e;char*p=mpfr_get_str(nullptr,&e,2,0,&x,NEAR);std::string s(p);mpfr_free_str(p);return s+"@"+std::to_string(e-static_cast<long>(s.size()-(s[0]=='-')));}
 std::string str(int rnd)const{if(!finite())return "nonfinite";long e;char*p=mpfr_get_str(nullptr,&e,10,0,&x,rnd);std::string s(p);mpfr_free_str(p);bool neg=s[0]=='-';if(neg)s.erase(0,1);return (neg?"-":"")+std::string("0.")+s+"e"+std::to_string(e);}
};
inline bool operator<(const M&a,const M&b){return mpfr_cmp(&a.x,&b.x)<0;}
inline bool operator<=(const M&a,const M&b){return mpfr_cmp(&a.x,&b.x)<=0;}
inline M op(const M&a,const M&b,int rnd,int(*f)(A18Mpfr*,const A18Mpfr*,const A18Mpfr*,int)){M v;f(&v.x,&a.x,&b.x,rnd);return v;}
struct I {M lo,hi;I(double x=0):lo(x),hi(x){if(!std::isfinite(x))nonfinite("BINARY64_INPUT");}I(M l,M h):lo(std::move(l)),hi(std::move(h)){} };
inline I operator+(const I&a,const I&b){return {op(a.lo,b.lo,DOWN,mpfr_add),op(a.hi,b.hi,UP,mpfr_add)};}
inline I operator-(const I&a,const I&b){return {op(a.lo,b.hi,DOWN,mpfr_sub),op(a.hi,b.lo,UP,mpfr_sub)};}
inline I operator-(const I&a){M l,h;mpfr_neg(&l.x,&a.hi.x,DOWN);mpfr_neg(&h.x,&a.lo.x,UP);return {l,h};}
inline I corners(const I&a,const I&b,int(*f)(A18Mpfr*,const A18Mpfr*,const A18Mpfr*,int)){M l=op(a.lo,b.lo,DOWN,f),h=op(a.lo,b.lo,UP,f);for(auto x:{&a.lo,&a.hi})for(auto y:{&b.lo,&b.hi}){M ll=op(*x,*y,DOWN,f),hh=op(*x,*y,UP,f);if(ll<l)l=ll;if(h<hh)h=hh;}return {l,h};}
inline I operator*(const I&a,const I&b){return corners(a,b,mpfr_mul);}
inline I operator/(const I&a,const I&b){if(b.lo<=M(0)&&M(0)<=b.hi)unsupported("DIVISION_DOMAIN");return corners(a,b,mpfr_div);}
inline I square(const I&a){M l=op(a.lo,a.lo,DOWN,mpfr_mul),l2=op(a.hi,a.hi,DOWN,mpfr_mul),h=op(a.lo,a.lo,UP,mpfr_mul),h2=op(a.hi,a.hi,UP,mpfr_mul);if(l2<l)l=l2;if(h<h2)h=h2;if(a.lo<=M(0)&&M(0)<=a.hi)l=M(0);return {l,h};}
inline bool less(const I&a,const I&b,bool equal=false){if(equal?a.hi<=b.lo:a.hi<b.lo)return true;if(equal?b.hi<a.lo:b.hi<=a.lo)return false;unsupported("INTERVAL_BRANCH_UNCERTAIN");}
inline I fun(const I&a,const std::string&f){M l,h;
 if(f=="sqrt"){if(a.lo<M(0))unsupported("SQRT_DOMAIN");mpfr_sqrt(&l.x,&a.lo.x,DOWN);mpfr_sqrt(&h.x,&a.hi.x,UP);}
 else if(f=="acos"){if(a.lo<M(-1)||M(1)<a.hi)unsupported("ACOS_DOMAIN");mpfr_acos(&l.x,&a.hi.x,DOWN);mpfr_acos(&h.x,&a.lo.x,UP);}
 else if(f=="tan"){if(a.lo<M(-1.5)||M(1.5)<a.hi)unsupported("TAN_DOMAIN");mpfr_tan(&l.x,&a.lo.x,DOWN);mpfr_tan(&h.x,&a.hi.x,UP);}
 else if(f=="sin"||f=="cos"){M mid=op(op(a.lo,a.hi,NEAR,mpfr_add),M(2),NEAR,mpfr_div);M rad=op(mid,a.lo,UP,mpfr_sub),r2=op(a.hi,mid,UP,mpfr_sub);if(rad<r2)rad=r2;auto fn=f=="sin"?mpfr_sin:mpfr_cos;fn(&l.x,&mid.x,DOWN);fn(&h.x,&mid.x,UP);l=op(l,rad,DOWN,mpfr_sub);h=op(h,rad,UP,mpfr_add);if(l<M(-1))l=M(-1);if(M(1)<h)h=M(1);}
 else unsupported("FUNCTION_"+f);
 if(!l.finite()||!h.finite())nonfinite("TRANSCENDENTAL");return {l,h};
}
using Vec=std::vector<I>;using Mat=std::vector<Vec>;
inline Vec add(const Vec&a,const Vec&b){if(a.size()!=b.size())unsupported("VECTOR_DIM");Vec r;for(size_t i=0;i<a.size();++i)r.push_back(a[i]+b[i]);return r;}
inline Vec sub(const Vec&a,const Vec&b){if(a.size()!=b.size())unsupported("VECTOR_DIM");Vec r;for(size_t i=0;i<a.size();++i)r.push_back(a[i]-b[i]);return r;}
inline Vec scale(const I&s,const Vec&a){Vec r;for(auto&x:a)r.push_back(s*x);return r;}
inline I dot(const Vec&a,const Vec&b){if(a.size()!=b.size())unsupported("DOT_DIM");I r;for(size_t i=0;i<a.size();++i)r=r+a[i]*b[i];return r;}
inline Mat tr(const Mat&a){Mat r(a[0].size(),Vec(a.size()));for(size_t i=0;i<a.size();++i)for(size_t j=0;j<a[0].size();++j)r[j][i]=a[i][j];return r;}
inline Vec mv(const Mat&a,const Vec&b){Vec r;for(auto&v:a)r.push_back(dot(v,b));return r;}
inline Mat mm(const Mat&a,const Mat&b){Mat bt=tr(b),r;for(auto&v:a)r.push_back(mv(bt,v));return r;}
inline Mat madd(const Mat&a,const Mat&b){Mat r;for(size_t i=0;i<a.size();++i)r.push_back(add(a[i],b[i]));return r;}
inline Mat ms(const I&s,const Mat&a){Mat r;for(auto&v:a)r.push_back(scale(s,v));return r;}
inline Mat skew(const Vec&v){I z=v[0]*0;return {{z,-v[2],v[1]},{v[2],z,-v[0]},{-v[1],v[0],z}};}
inline I norm(const Vec&v){I r;for(auto&x:v)r=r+square(x);return fun(r,"sqrt");}
inline Mat rot(const Mat&a){Mat r;for(int i=0;i<3;++i)r.emplace_back(a[i].begin(),a[i].begin()+3);return r;}
inline Vec pos(const Mat&a){return {a[0][3],a[1][3],a[2][3]};}
inline Vec slice(const Vec&a,int b,int e){return Vec(a.begin()+b,a.begin()+e);}
inline Vec cat(Vec a,const Vec&b){a.insert(a.end(),b.begin(),b.end());return a;}
inline I paired(const Vec&a,const Vec&b){I r;for(size_t i=0;i<a.size();++i)r=r+(a[i]-b[i])*(a[i]+b[i])/2;return r;}
struct Decision {std::string status,reason;double fidelity=0;I ratio;};
inline Decision choose(const I&P,const I&D,double threshold=.001){
 if(!P.lo.finite()||!P.hi.finite()||!D.lo.finite()||!D.hi.finite())return {"NUMERIC_REFERENCE_NONFINITE","BOUNDS"};
 // Exact decimal width comparison: multiply by 10^15 (exact integer at 333 bits).
 for(auto*q:{&P,&D})if(M(2)<op(op(q->hi,q->lo,UP,mpfr_sub),M(1e15),UP,mpfr_mul))return {"NUMERIC_REDUCTION_UNRESOLVED","WIDTH"};
 if(P.hi<=M(0)||D.hi<=M(0))return {"REJECT","CERTIFIED_NON_DESCENT"};
 if(P.lo<=M(0)||D.lo<=M(0))return {"NUMERIC_REDUCTION_UNRESOLVED","SIGN"};
 I ratio(op(D.lo,P.hi,DOWN,mpfr_div),op(D.hi,P.lo,UP,mpfr_div));
 if(ratio.hi<=M(threshold))return {"REJECT","CERTIFIED_LOW_FIDELITY",0,ratio};
 if(ratio.lo<=M(threshold))return {"NUMERIC_REDUCTION_UNRESOLVED","FIDELITY",0,ratio};
 double q=ratio.lo.d(DOWN);if(!std::isfinite(q))return {"NUMERIC_REFERENCE_NONFINITE","FIDELITY_CONVERSION",0,ratio};
 if(!(q>threshold)||ratio.lo<M(q))return {"NUMERIC_REDUCTION_UNRESOLVED","FIDELITY_CONVERSION",0,ratio};
 return {"ACCEPT","CERTIFIED_DESCENT_AND_FIDELITY",q,ratio};
}
inline std::string exceptionStatus(const std::exception&e){if(auto*f=dynamic_cast<const Failure*>(&e))return f->status;std::string s=e.what();for(auto*p:{"NUMERIC_REFERENCE_NONFINITE","NUMERIC_REDUCTION_UNRESOLVED","NUMERIC_REFERENCE_UNSUPPORTED"})if(s.find(p)!=std::string::npos)return p;return "NUMERIC_REFERENCE_UNSUPPORTED";}
}
