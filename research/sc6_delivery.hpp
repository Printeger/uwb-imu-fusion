#pragma once
#include <cstdint>
namespace uwb_imu_pl::research {
// Receiver owns validity at its actual clock, independent of worker service time.
class Sc6Delivery {
 uint64_t epoch_=0;bool gap_=false,protected_=false;
 public:
 void sourceGap(){gap_=true;protected_=false;}
 bool accept(uint64_t epoch,int64_t stamp,uint64_t version,bool spatial,
             double sink_from_measurement_ms){
  protected_=false;if(gap_ || epoch!=epoch_+1 || stamp!=int64_t(epoch)*50000000LL || version!=epoch+1)return false;
  epoch_=epoch;protected_=spatial && sink_from_measurement_ms>=0 && sink_from_measurement_ms<=50;return protected_;
 }
 bool finalDeadline(double age){protected_&=age>=0 && age<=50;return protected_;}
 bool currentProtected()const{return protected_;}
};
}
