#pragma once

#include <string>

namespace uifgo {

// Lower-case, unprefixed SHA-256 hex digests. Callers add a schema-specific
// prefix so the algorithm and the hashed context remain explicit.
std::string Sha256Hex(const std::string& bytes);
std::string Sha256FileHex(const std::string& path);

}  // namespace uifgo
