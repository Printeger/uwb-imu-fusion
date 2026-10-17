uint64 epoch
uint64 batch_id
int64 sample_ns
int64 latest_original_used_ns
int64 callback_steady_ns
int64 worker_start_steady_ns
int64 publish_steady_ns
int64 replay_origin_steady_ns
int64 recording_origin_ns
float64 core_ms
uint64 input_queue_high_water
uint64 input_events
# Complete canonical output record, including state, action, risk and digests.
# Diagnostic research output; never a protected production navigation message.
string record
bool finished
string error
