#include "canonical_wire.hpp"
#include "wire_result.hpp"
#include <tbb/global_control.h>
#include <fstream>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
int main(int argc,char** argv){try{
 if(argc!=6)throw std::invalid_argument("usage: sc9_wire_runner MODEL ANCHORS EVENTS_JSONL OUTPUT_CSV on|off");
 const std::string mode=argv[5];if(mode!="on" && mode!="off")throw std::invalid_argument("WIRE_MODE");
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 auto setup=empiricalSetup(YAML::LoadFile(argv[1]));CanonicalWireAdapter adapter(setup,wireAnchors(YAML::LoadFile(argv[2]),setup));
 std::ifstream input(argv[3]);std::ofstream output(argv[4]);if(!input || !output)throw std::runtime_error("WIRE_FILE_OPEN");output<<wireResultHeader();
 std::string line;while(std::getline(input,line)){
  if(line.size()>1024*1024)throw std::length_error("WIRE_LINE_CAPACITY");
  const auto event=parseWireEvent(YAML::Load(line));adapter.append(event);
  for(;;){const auto start=std::chrono::steady_clock::now();auto emission=adapter.next(mode=="on");if(!emission)break;
   const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();output<<wireResultRecord(emission->result,emission->batch,adapter.stream().epochs(),ms);if(!output)throw std::runtime_error("WIRE_OUTPUT_FAILED");
  }
 }
 if(!input.eof())throw std::runtime_error("WIRE_INPUT_READ_FAILED");
 const auto& s=adapter.stream();std::cerr<<"events="<<adapter.events()<<" imu="<<s.receivedImu()<<" ranges="<<s.receivedBatches()<<" startup_batches="<<s.startupBatches()<<" commits="<<s.epochs()<<" pending_batches="<<s.pendingBatches()<<" unused_imu="<<s.pendingImu()<<" backend_updates="<<s.updates()<<" startup_digest="<<s.startupDigest()<<'\n';
 if(!s.initialized() || s.pendingBatches())throw std::runtime_error("WIRE_END_WITH_UNCLOSED_BATCHES");return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
