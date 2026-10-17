# SC9 opt-in lossless range transport. stamp/received are signed original ns.
uint64 measurement_id
uint64 factor_id
uint64 anchor_id
uint64 tag_id
uint64 source_sequence
int64 stamp_ns
int64 received_ns
uint64 calibration_version
uint32 quality_flags
bool valid
float64 range_m
float64 raw_range_m
float64 sigma_m
float64[3] anchor
float64[3] lever
