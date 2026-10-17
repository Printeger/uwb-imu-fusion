#include "sc9_paced_replay.hpp"
#include <iostream>
using namespace uwb_imu_pl::research;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
template<class F>void refuses(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"expected paced refusal");}
template<class F>void await(F f){auto limit=Sc9PacedReleases::Clock::now()+std::chrono::seconds(2);while(!f()){require(Sc9PacedReleases::Clock::now()<limit,"producer did not independently progress");std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
int main(){try{
 refuses([]{Sc9PacedReleases bad(0,{1,1});});
 refuses([]{Sc9PacedReleases bad(-1,{1});});
 refuses([]{Sc9PacedReleases bad(0,{1000000000000001LL});});
 {Sc9PacedReleases source(0,{1000000,2000000,3000000});
  await([&]{return source.pendingCount()==3;});const auto observed=Sc9PacedReleases::Clock::now();
  for(size_t i=1;i<=3;++i){const auto tick=source.next(i);require(tick.end==int64_t(i)*1000000&&tick.released>=tick.target&&tick.released<=observed,"release depended on consumer or occurred early");require(tick.high_water==i,"queue peak missing");}
  refuses([&]{source.next(4);});
 }
 {std::vector<int64_t> ends;for(int64_t i=1;i<=513;++i)ends.push_back(i);
  Sc9PacedReleases source(0,std::move(ends));await([&]{return source.overflowed();});
  require(source.pendingCount()==512,"bounded queue changed capacity");refuses([&]{source.next(1);});
 }
 const auto before=Sc9PacedReleases::Clock::now();{Sc9PacedReleases cancelled(0,{2000000000});}
 require(Sc9PacedReleases::Clock::now()-before<std::chrono::milliseconds(500),"cancellation waited for a future release");
 std::cout<<"SC9 paced release PASS independent producer, absolute order, bounded queue refusal, missing suffix and prompt cancellation\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
