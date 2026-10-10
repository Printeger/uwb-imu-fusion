#include "gaussian_reference.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
using namespace uwb_imu_pl::research;
void check(bool x){if(!x)throw std::runtime_error("Gaussian oracle check failed");}
int main(int argc,char** argv){
 if(argc!=2)throw std::runtime_error("usage: fde_next_checks config/fde_next.yaml");
 auto cfg=YAML::LoadFile(argv[1]);
 const double task=2*cfg["noise_tail_per_candidate"].as<double>()*cfg["max_candidates"].as<int>()*cfg["epochs"].as<int>();
 check(task<cfg["risk_budget"].as<double>());
 std::cout<<"model="<<cfg["model_version"].as<std::string>()<<" task_union_risk="<<task<<"\n";
 Eigen::MatrixXd a(5,2);a<<1,0,0,1,1,1,2,-1,-1,2;
 Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,2);c.topRows(2).setIdentity();
 Eigen::VectorXd z(5);z<<.12,-.08,.01,.03,-.02;
 Eigen::MatrixXd sigma=Eigen::MatrixXd::Identity(5,5);sigma(0,1)=sigma(1,0)=.4;
 Eigen::MatrixXd b=Eigen::MatrixXd::Zero(5,1);b(4,0)=1;
 auto r=gaussianReference(a,z,sigma,c,{b},1e-3,1e-6);
 check(r.status=="FROZEN_MODEL_ONLY" && r.pl.allFinite());
 auto exact=(a.transpose()*sigma.ldlt().solve(a)).ldlt().solve(a.transpose()*sigma.ldlt().solve(z)).eval();
 check((r.mean-exact).norm()<1e-12);
 auto wrong=gaussianReference(a,z,Eigen::MatrixXd::Identity(5,5),c,{b},1e-3,1e-6);
 check((wrong.mean-r.mean).norm()>1e-4);
 Eigen::MatrixXd safe_c=Eigen::MatrixXd::Zero(3,2);safe_c(0,1)=1;
 check(gaussianReference(a,z,sigma,safe_c,{a.col(0)},1e-3,1e-6).pl.allFinite());
 check(gaussianReference(a,z,sigma,c,{a.col(0)},1e-3,1e-6).status=="DANGEROUS_NULLSPACE");
 check(gaussianReference(a,z,sigma,c,{1e-10*b},1e-3,1e-6).status=="INDETERMINATE");
 std::vector<int> rows{1,2,3,4};check(principal(sigma,rows)(0,1)==0);
 Eigen::MatrixXd selected_a(4,2);Eigen::VectorXd selected_z(4);
 for(int i=0;i<4;++i){selected_a.row(i)=a.row(rows[i]);selected_z(i)=z(rows[i]);}
 auto deleted=gaussianReference(selected_a,selected_z,principal(sigma,rows),c,{},1e-3,1e-6);
 auto wa=(r.whitener*a).eval();auto wz=(r.whitener*z).eval();
 for(int i=0;i<4;++i){selected_a.row(i)=wa.row(rows[i]);selected_z(i)=wz(rows[i]);}
 auto mixed=gaussianReference(selected_a,selected_z,Eigen::MatrixXd::Identity(4,4),c,{},1e-3,1e-6);
 check((deleted.mean-mixed.mean).norm()>1e-4);
 Eigen::MatrixXd j(4,1);j<<1,2,3,4;auto joint=(j*j.transpose()).eval();check(joint(0,3)==4);
 check((principal(joint,rows={0,2})-(Eigen::Vector2d(1,3)*Eigen::Vector2d(1,3).transpose())).norm()==0);
 // Independent body->tag right-local rotation derivative.
 Eigen::Vector3d lever(.2,-.1,.08);Eigen::Matrix3d rot=Eigen::AngleAxisd(.3,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
 Eigen::Matrix3d skew;skew<<0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;
 for(int j=0;j<3;++j){Eigen::Vector3d axis=Eigen::Vector3d::Unit(j);
 auto fd=(rot*Eigen::AngleAxisd(1e-6,axis).toRotationMatrix()*lever-rot*Eigen::AngleAxisd(-1e-6,axis).toRotationMatrix()*lever)/2e-6;
 check((fd+rot*skew.col(j)).norm()<1e-9);}
 check(riskFloor(3e-5,1e-7,{1e-4},.001,.001,1000)>4e-5);
 check(riskFloor(1e-6,0,{1e-4},.001,.001,9)<4e-5);
 std::cout<<"checks=16 finite_pl="<<r.pl.transpose()<<" statistic="<<r.statistic<<" threshold="<<r.threshold<<"\n";
}
