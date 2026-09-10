#include "uifgo/hash_utils.h"

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace uifgo {
namespace {

class Sha256 {
 public:
  void Update(const void* data, size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < size; ++i) {
      buffer_[buffer_size_++] = bytes[i];
      if (buffer_size_ == buffer_.size()) Transform();
      bit_count_ += 8;
    }
  }

  std::string FinalHex() {
    buffer_[buffer_size_++] = 0x80;
    if (buffer_size_ > 56) {
      while (buffer_size_ < 64) buffer_[buffer_size_++] = 0;
      Transform();
    }
    while (buffer_size_ < 56) buffer_[buffer_size_++] = 0;
    for (int shift = 56; shift >= 0; shift -= 8)
      buffer_[buffer_size_++] =
          static_cast<unsigned char>((bit_count_ >> shift) & 0xff);
    Transform();
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::uint32_t word : state_) out << std::setw(8) << word;
    return out.str();
  }

 private:
  static std::uint32_t Rotate(std::uint32_t value, unsigned count) {
    return (value >> count) | (value << (32 - count));
  }
  void Transform() {
    static constexpr std::array<std::uint32_t, 64> k = {{
      0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
      0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
      0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
      0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
      0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
      0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
      0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
      0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2}};
    std::array<std::uint32_t, 64> w{};
    for (size_t i = 0; i < 16; ++i)
      w[i] = (static_cast<std::uint32_t>(buffer_[4*i]) << 24) |
             (static_cast<std::uint32_t>(buffer_[4*i+1]) << 16) |
             (static_cast<std::uint32_t>(buffer_[4*i+2]) << 8) |
             static_cast<std::uint32_t>(buffer_[4*i+3]);
    for (size_t i = 16; i < 64; ++i) {
      const auto s0 = Rotate(w[i-15],7) ^ Rotate(w[i-15],18) ^ (w[i-15] >> 3);
      const auto s1 = Rotate(w[i-2],17) ^ Rotate(w[i-2],19) ^ (w[i-2] >> 10);
      w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    auto a=state_[0], b=state_[1], c=state_[2], d=state_[3];
    auto e=state_[4], f=state_[5], g=state_[6], h=state_[7];
    for (size_t i = 0; i < 64; ++i) {
      const auto s1 = Rotate(e,6) ^ Rotate(e,11) ^ Rotate(e,25);
      const auto choice = (e & f) ^ (~e & g);
      const auto t1 = h + s1 + choice + k[i] + w[i];
      const auto s0 = Rotate(a,2) ^ Rotate(a,13) ^ Rotate(a,22);
      const auto majority = (a & b) ^ (a & c) ^ (b & c);
      const auto t2 = s0 + majority;
      h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
    state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
    buffer_size_ = 0;
  }
  std::array<std::uint32_t,8> state_{{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                                      0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19}};
  std::array<unsigned char,64> buffer_{};
  size_t buffer_size_ = 0;
  std::uint64_t bit_count_ = 0;
};

}  // namespace

std::string Sha256Hex(const std::string& bytes) {
  Sha256 hash;
  hash.Update(bytes.data(), bytes.size());
  return hash.FinalHex();
}

std::string Sha256FileHex(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot hash " + path);
  Sha256 hash;
  std::array<char, 8192> buffer{};
  while (in) {
    in.read(buffer.data(), buffer.size());
    hash.Update(buffer.data(), static_cast<size_t>(in.gcount()));
  }
  if (!in.eof()) throw std::runtime_error("failed while hashing " + path);
  return hash.FinalHex();
}

}  // namespace uifgo
