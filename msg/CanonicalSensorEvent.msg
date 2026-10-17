# One ordered channel preserves original cross-sensor receive order.
uint8 IMU=1
uint8 RANGE=2
uint8 END=3
uint8 kind
uint64 event_sequence
int64 stamp_ns
int64 received_ns
uint64 measurement_id
uint32 quality_flags
float64[3] accel
float64[3] gyro
uint64 batch_id
CanonicalRange[] ranges
# Independent producer schedule; steady clock, same host only.
int64 release_steady_ns
int64 replay_origin_steady_ns
int64 recording_origin_ns
