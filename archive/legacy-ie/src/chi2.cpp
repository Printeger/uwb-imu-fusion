#include "uifgo/optimizer.h"
#include <cmath>
#include <cstddef>

namespace uifgo {

double Chi2inv(double p, size_t dof) {
  // Lookup table for common degrees of freedom (p=0.95 and p=0.99)
  static const double t95[7] = {0.0, 3.8415, 5.9915, 7.8147,
                                     9.4877, 11.0705, 12.5916};
  static const double t99[7] = {0.0, 6.6349, 9.2103, 11.3449,
                                     13.2767, 15.0863, 16.8119};

  if (dof >= 1 && dof <= 6) {
    return (p >= 0.99) ? t99[dof] : t95[dof];
  }

  // Wilson-Hilferty approximation for larger dof
  double z = (p >= 0.99) ? 2.3263 : 1.6449;  // standard normal quantile
  double d = static_cast<double>(dof);
  double c = 1.0 - 2.0 / (9.0 * d) + z * std::sqrt(2.0 / (9.0 * d));
  return d * c * c * c;
}

}  // namespace uifgo
