#include "sc6_provider.hpp"
#include "uwb_imu_pl/common/runtime_route_audit.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "native_semantic_audit.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
int main(){try{
 SemanticDigest encoding;encoding.u(1);encoding.s("x");encoding.d(.5);
 if(encoding.finish()!="21168339ac726c715d00f0e558e620bae5ee4d5b3c84f67d51bdd6cdbd9bb560")
  throw std::runtime_error("CANONICAL_ENCODING_PYTHON_HASHLIB_ORACLE");
 CommitReceipt receipt;receipt.transaction_id=TransactionId(77);receipt.added_factor_slots={2,3};
 auto reference=receiptDigest(receipt);receipt.added_factor_slots[1]=4;
 if(receiptDigest(receipt)==reference)throw std::runtime_error("RECEIPT_SLOT_MUTATION_NOT_BOUND");
 Sc6Provider source("config/sc7.yaml","S0",20261010,1);
 auto before=runtimeRouteAuditV1();
 IncrementalUwbImuEstimator backend(source.cfg,source.lever);
 backend.initialize(source.initial,source.cfg.realtime.prior_sigmas);
 const auto initial_digest=ledgerDigest(backend.factorLedger());
 if(initial_digest.size()!=64 || initial_digest!=ledgerDigest(backend.factorLedger()))
  throw std::runtime_error("NATIVE_LEDGER_DIGEST_NOT_DETERMINISTIC");
 RealtimeIntegrityPipeline old(&backend,IntegrityMonitor(RiskBudget{},1e-9,1e12));
 // Deliberately incomplete input: this positive instrumentation control tests
 // entry into the old path, not its numerical recovery on malformed sensors.
 UwbBatch batch;batch.id=BatchId(1);batch.timestamp=TimestampNs(50000000);
 try{old.processUwbBatch(batch);}catch(const std::exception&){}
 const auto midway=runtimeRouteAuditV1();
 if(midway.incremental_backend_constructions!=before.incremental_backend_constructions+1 ||
    midway.old_pipeline_constructions!=before.old_pipeline_constructions+1 ||
    midway.old_engine_calls!=before.old_engine_calls+1)
  throw std::runtime_error("OLD_ROUTE_INSTRUMENTATION_NOT_OBSERVED");
 std::vector<CandidateEvaluation> candidates;
 try{FdeManager{}.decide(DetectorResultV2{}, {}, {}, &candidates, {}, RiskBudgetV2{});}catch(const std::exception&){}
 if(runtimeRouteAuditV1().old_fde_calls!=midway.old_fde_calls+1)
  throw std::runtime_error("OLD_FDE_INSTRUMENTATION_NOT_OBSERVED");
 std::cout<<"SC9 route counters PASS actual old pipeline construction/call, old FDE call and native backend construction observed\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
