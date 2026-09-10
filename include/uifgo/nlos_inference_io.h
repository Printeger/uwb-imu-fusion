#pragma once

#include <string>
#include <vector>

#include "uifgo/nlos_inference.h"

namespace uifgo {

struct InferenceArtifactWriteResult {
  std::vector<std::string> files;
};

// Writes the complete T08 artifact set from one InferenceResult.  No Stage-2
// graph/Values, plan, Config, oracle input, or evaluation truth is accepted by
// this interface, which makes mixed-result exports structurally impossible.
InferenceArtifactWriteResult WriteInferenceArtifacts(
    const std::string& output_directory, const InferenceResult& result);

}  // namespace uifgo
