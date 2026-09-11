// Independent diagnostic binary. Reuse the exact existing loader/export helpers
// without exposing any hold-out/truth argument in the production executable.
#define Open OriginalPaperOpen
#define main uifgo_original_paper_main
#include "run_ie_paper.cpp"
#undef main
#undef Open
#include "uifgo/fde_math.h"
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace {
std::ofstream Open(const fs::path& p) { auto out=OriginalPaperOpen(p);out<<std::setprecision(17);return out;}
struct Prepared {
 uifgo::Config cfg;uifgo::PaperInputPlan plan;gtsam::NonlinearFactorGraph graph;
 gtsam::Values initial;std::vector<uifgo::FactorMeta> meta;uifgo::FdeOptions options;
 uifgo::FdeResult fde;std::string imu_hash;
};
Prepared Prepare(const fs::path& config) {
 Prepared p;p.cfg=uifgo::ConfigLoader::Load(config.string());ValidateSupportedConfig(p.cfg);
 std::vector<uifgo::ImuSample> imu;std::vector<uifgo::UwbFrame> raw;
 auto loaded=LoadData(config.parent_path().string(),&p.cfg,&imu,&raw,config.string());
 p.plan=uifgo::BuildPaperInputPlan(raw,p.cfg,loaded.recording_id);
 auto keyframes=uifgo::MaterializePaperKeyframes(p.plan,uifgo::AllPlannedObservationMask(p.plan));
 uifgo::ValidatePaperFixedBeta(p.plan,p.cfg);
 auto init=uifgo::Initializer(p.cfg).Run(imu,keyframes);if(!init.ok)throw std::runtime_error("INITIALIZER_FAILED");
 uifgo::GraphBuilder builder(p.cfg,p.cfg.paper_imu_covariance_model);std::vector<size_t> ids;
 builder.Build(keyframes,imu,init,&p.graph,&p.initial,&ids);p.meta=builder.factor_meta();
 if(uifgo::ReplacePosePriorsForPaperPath(&p.graph)!=1)throw std::runtime_error("PRIOR_COUNT");
 p.options.chi2_probability=p.cfg.chi2_reject_prob;p.options.gap_threshold_s=p.cfg.discovery_gap_threshold_s;
 p.options.minimum_count=p.cfg.discovery_short_min_count;p.options.minimum_duration_s=p.cfg.discovery_short_min_duration_s;
 p.options.preliminary_lm.max_iterations=p.cfg.lm_max_iter;p.options.preliminary_lm.relative_tolerance=p.cfg.lm_rel_tol;p.options.preliminary_lm.absolute_tolerance=p.cfg.lm_abs_tol;
 return p;
}
uifgo::InferenceContentIdentity Identity(const gtsam::NonlinearFactorGraph& g,const gtsam::Values& v){
 uifgo::InferenceIdentityContext c;c.input_sha256=c.config_sha256=c.input_plan_sha256=c.support_partition_sha256=c.calibration_sha256=c.solver_config_sha256="FORENSIC_ONLY";
 return uifgo::ComputeInferenceContentIdentity(g,v,c);
}
void Matrix(const fs::path& path,const Eigen::MatrixXd& m){auto out=Open(path);out<<std::setprecision(17);for(int i=0;i<m.rows();++i){for(int j=0;j<m.cols();++j){if(j)out<<',';out<<m(i,j);}out<<'\n';}}
struct Linear {Eigen::SparseMatrix<double>A;Eigen::VectorXd e;std::vector<size_t> offsets;};
Linear Linearize(const gtsam::NonlinearFactorGraph& g,const gtsam::Values& v,const fs::path& dir){
 Linear l;auto gf=g.linearize(v);gtsam::Ordering order;size_t column=0;
 auto keys=Open(dir/"columns.csv");keys<<"key,column,dimension\n";for(const auto& x:v){order.push_back(x.key);keys<<x.key<<','<<column<<','<<x.value.dim()<<'\n';column+=x.value.dim();}
 size_t nr=0,nc=0;auto entries=gf->sparseJacobian(order,nr,nc);if(nc!=v.dim()+1)throw std::runtime_error("COLUMN_MAPPING");
 l.e=Eigen::VectorXd::Zero(nr);std::vector<Eigen::Triplet<double>> triplets;
 auto sparse=Open(dir/"A_triplets.csv");sparse<<"row,column,value\n";
 for(const auto& t:entries){int r=std::get<0>(t),c=std::get<1>(t);double a=std::get<2>(t);if(c==int(nc-1))l.e[r]=-a;else{triplets.emplace_back(r,c,a);sparse<<r<<','<<c<<','<<a<<'\n';}}
 l.A.resize(nr,nc-1);l.A.setFromTriplets(triplets.begin(),triplets.end());
 auto rows=Open(dir/"rows.csv");rows<<"factor_index,row,dimension,keys\n";size_t offset=0;
 for(size_t i=0;i<g.size();++i){l.offsets.push_back(offset);size_t n=g.at(i)->dim();rows<<i<<','<<offset<<','<<n<<",\"";for(auto k:g.at(i)->keys())rows<<k<<';';rows<<"\"\n";
 auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(g.at(i));Eigen::VectorXd e=f->noiseModel()->whiten(f->unwhitenedError(v));
 if((e-l.e.segment(offset,n)).norm()>1e-12+1e-10*e.norm())throw std::runtime_error("PHYSICAL_WHITENING_ROW_MISMATCH");
 offset+=n;}
 if(offset!=nr)throw std::runtime_error("ROW_COVERAGE");
 Matrix(dir/"whitened_residual.csv",l.e);return l;
}
void TestJson(std::ostream& out,const uifgo::FdeQuadraticTest&t){out<<"{\"valid\":"<<(t.valid?"true":"false")<<",\"reason\":\""<<t.reason<<"\",\"rank\":"<<t.rank<<",\"statistic\":"<<t.statistic<<",\"threshold\":"<<t.threshold<<",\"rank_threshold\":"<<t.rank_threshold<<",\"null_component_norm\":"<<t.null_component_norm<<",\"rejected\":"<<(t.rejected?"true":"false")<<'}';}
void Trajectory(const fs::path& path,const Prepared&p,const gtsam::Values&v){auto out=Open(path);out<<"keyframe,time,x,y,z\n";for(size_t k=0;k<p.plan.keyframes.size();++k){auto x=v.at<gtsam::Pose3>(X(k)).translation();out<<k<<','<<p.plan.keyframes[k].sensor_time<<','<<x.x()<<','<<x.y()<<','<<x.z()<<'\n';}}
uifgo::FdeProjectionBlock Analyze(const Prepared&p,const gtsam::NonlinearFactorGraph&g,const gtsam::Values&v,const std::set<uint64_t>&held,const fs::path&dir){
 fs::create_directory(dir);auto l=Linearize(g,v,dir);std::vector<size_t> rows;for(const auto&m:p.meta)if(held.count(m.obs_id))rows.push_back(l.offsets.at(m.factor_index));
 auto block=uifgo::FdeFullProjectionBlock(l.A,l.e,rows);auto out=Open(dir/"statistics.json");
 out<<"{\"rows\":"<<l.A.rows()<<",\"columns\":"<<l.A.cols()<<",\"certificate_valid\":"<<(block.certificate.valid?"true":"false")<<",\"certificate_reason\":\""<<block.certificate.reason<<"\"";
 if(!block.certificate.valid){out<<'}';return block;}
 Matrix(dir/"Pww.csv",block.Pww);Matrix(dir/"response.csv",block.response);Matrix(dir/"projected_residual.csv",block.projected_residual);
 Eigen::VectorXd ew(rows.size()),pw(rows.size());for(size_t j=0;j<rows.size();++j){ew[j]=l.e[rows[j]];pw[j]=block.projected_residual[rows[j]];}
 Matrix(dir/"window_raw_whitened.csv",ew);Matrix(dir/"window_projected.csv",pw);
 int dof=l.A.rows()-block.certificate.rank;
 out<<",\"global_dof\":"<<dof<<",\"global_raw_statistic\":"<<l.e.squaredNorm()<<",\"global_projected_statistic\":"<<block.projected_residual.squaredNorm()<<",\"global_threshold\":"<<uifgo::FdeChiSquareQuantile(dof)<<",\"tangent_residual_norm\":"<<(l.e-block.projected_residual).norm()<<",\"symmetry_error\":"<<block.symmetry_error<<",\"idempotence_error\":"<<block.idempotence_error<<",\"orthogonality_error\":"<<block.orthogonality_error<<",\"window_raw\":";TestJson(out,uifgo::FdeCovarianceTest(ew,block.Pww));out<<",\"window_projected\":";TestJson(out,uifgo::FdeCovarianceTest(pw,block.Pww));out<<"}\n";return block;
}
}
int main(int argc,char**argv){
 try{
 if(argc!=5)throw std::invalid_argument("usage: fde_forensics CLEAN_CONFIG INJECTED_CONFIG HOLDOUT_TRUTH OUTPUT_NEW_DIR");
 fs::path root=fs::absolute(argv[4]);if(fs::exists(root))throw std::runtime_error("OUTPUT_EXISTS");fs::create_directories(root);
 auto truth=YAML::LoadFile(argv[3]);std::set<uint64_t> held;for(const auto&id:truth["affected_obs_ids"])held.insert(id.as<uint64_t>());
 auto clean=Prepare(fs::absolute(argv[1]));auto injected=Prepare(fs::absolute(argv[2]));
 if(held.size()!=111 || clean.meta.size()!=injected.meta.size() || Identity(clean.graph,clean.initial).values_sha256!=Identity(injected.graph,injected.initial).values_sha256)throw std::runtime_error("PAIR_INITIAL_OR_COUNT_MISMATCH");
 std::set<size_t> removed;double max_measurement_error=0,max_prefit_error=0;size_t count=0;
 auto audit=Open(root/"physical_pair.csv");audit<<"obs_id,factor_index,keys,clean_measurement,injected_measurement,clean_prefit,injected_prefit,sigma,held\n";
 for(size_t j=0;j<clean.meta.size();++j){const auto&a=clean.meta[j];const auto&b=injected.meta[j];if(a.obs_id!=b.obs_id || a.factor_index!=b.factor_index || a.keys!=b.keys)throw std::runtime_error("META_PAIR_MISMATCH");
 auto fc=boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(clean.graph.at(a.factor_index));auto fi=boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(injected.graph.at(b.factor_index));if(!fc||!fi)throw std::runtime_error("PHYSICAL_FACTOR_TYPE");
 double shift=held.count(a.obs_id)?.5:0.;double rc=fc->unwhitenedError(clean.initial)[0],ri=fi->unwhitenedError(injected.initial)[0];
 if(fc->noiseModel()->sigmas()!=fi->noiseModel()->sigmas())throw std::runtime_error("NOISE_PAIR_MISMATCH");
 max_measurement_error=std::max(max_measurement_error,std::abs(fi->measured()-fc->measured()-shift));max_prefit_error=std::max(max_prefit_error,std::abs(ri-rc+shift));
 double tol=16*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(rc),std::abs(ri),std::abs(fc->measured()),std::abs(fi->measured())});
 if(std::abs(fi->measured()-fc->measured()-shift)>tol || std::abs(ri-rc+shift)>tol)throw std::runtime_error("INJECTION_OR_SIGN_FAILED");
 if(shift){removed.insert(a.factor_index);++count;}audit<<a.obs_id<<','<<a.factor_index<<",\"";for(auto k:a.keys)audit<<k<<';';audit<<"\","<<fc->measured()<<','<<fi->measured()<<','<<rc<<','<<ri<<','<<fc->noiseModel()->sigmas()[0]<<','<<(shift!=0)<<'\n';}
 if(count!=30)throw std::runtime_error("PLANNED_TRUTH_COUNT_MISMATCH");
 gtsam::NonlinearFactorGraph sc,si;for(size_t i=0;i<clean.graph.size();++i)if(!removed.count(i)){sc.push_back(clean.graph.at(i));si.push_back(injected.graph.at(i));}
 if(Identity(sc,clean.initial).graph_linearization_sha256!=Identity(si,injected.initial).graph_linearization_sha256)throw std::runtime_error("SUBSET_GRAPH_MISMATCH");
 {auto out=Open(root/"pair_audit.json");out<<"{\"affected_raw\":"<<held.size()<<",\"affected_planned\":"<<count<<",\"max_measurement_error_m\":"<<max_measurement_error<<",\"max_prefit_error_m\":"<<max_prefit_error<<",\"initial_values_identity\":\""<<Identity(sc,clean.initial).values_sha256<<"\",\"subset_graph_identity\":\""<<Identity(sc,clean.initial).graph_linearization_sha256<<"\",\"subset_identical\":true}\n";}
 for(auto item:{std::make_pair("clean",&clean),std::make_pair("injected",&injected)}){
 auto&p=*item.second;fs::path dir=root/item.first;fs::create_directory(dir);WriteObservations(dir,p.plan,uifgo::AllPlannedObservationMask(p.plan));
 uifgo::FdeContext c;c.input_plan_hash=p.plan.plan_sha256;c.source_hash=c.config_hash=c.calibration_hash=c.solver_config_hash=c.common_preparation_id=c.physical_graph_hash=c.initial_values_hash="FORENSIC_ONLY";
 p.fde=uifgo::ImuAidedFdeSupportProvider(p.options).Run(p.graph,p.initial,p.meta,p.plan,p.cfg,c);WriteFdeArtifacts(dir,p.fde,p.options);
 if(!p.fde.success())throw std::runtime_error(std::string(item.first)+":"+p.fde.reason);
 Trajectory(dir/"trajectory.csv",p,p.fde.reference.values);Analyze(p,p.graph,p.fde.reference.values,held,dir/"linear");
 }
 auto subset=uifgo::RunPreliminaryTightlyCoupledLm(sc,clean.initial,clean.options.preliminary_lm);
 {auto out=Open(root/"subset_status.json");out<<"{\"converged\":"<<(subset.converged?"true":"false")<<",\"reason\":\""<<subset.reason<<"\",\"iterations\":"<<subset.iterations<<"}\n";}
 if(!subset.converged)return 1;
 Trajectory(root/"subset_trajectory.csv",clean,subset.values);
 // Full physical model relinearized at subset Values provides Pww; Pww^-1
 // is held-out covariance in whitened units, including prediction uncertainty.
 Analyze(clean,clean.graph,subset.values,held,root/"subset_prediction_linear");
 auto residuals=Open(root/"held_out.csv");residuals<<"obs_id,clean_residual,injected_residual,sigma\n";
 for(const auto&m:clean.meta)if(held.count(m.obs_id)){auto fc=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(clean.graph.at(m.factor_index));auto fi=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(injected.graph.at(m.factor_index));residuals<<m.obs_id<<','<<fc->unwhitenedError(subset.values)[0]<<','<<fi->unwhitenedError(subset.values)[0]<<','<<fc->noiseModel()->sigmas()[0]<<'\n';}
 // Actual nonlinear full/subset differences in each full solution's tangent.
 for(auto item:{std::make_pair("clean",&clean),std::make_pair("injected",&injected)}){
 const auto&v=item.second->fde.reference.values;Eigen::VectorXd delta(v.dim());size_t o=0;for(const auto&x:v){auto d=x.value.localCoordinates_(subset.values.at(x.key));delta.segment(o,d.size())=d;o+=d.size();}Matrix(root/item.first/"subset_delta.csv",delta);}
 return 0;
 }catch(const std::exception&e){std::cerr<<"FORENSIC_FAILED: "<<e.what()<<'\n';return 1;}
}
