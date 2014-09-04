#ifndef FLIR_GIGE_PLANCK_H_
#define FLIR_GIGE_PLANCK_H_

#include <cmath>

namespace flir_gige {

/**
 * @brief The Planck constants from flir thermal camera
 */
struct Planck {
  Planck() = default;
  Planck(const double B, const double F, const double O, const double R)
      : B{B}, F{F}, O{O}, R{R} {}

  double B;
  double F;
  double O;
  double R;
  static const double kT0{273.15};  ///< Kelvin at 0 celcius

  /**
   * @brief CelsiusToRaw Convert celsius to 16-bit raw data
   * @param t Celcius
   * @return raw data
   */
  inline int CelsiusToRaw(const double t) const {
    return R / (std::exp(B / (t + kT0)) - F) + O;
  }

  /**
   * @brief RawToCelsius Convert 16-bit raw data to celsius
   * @param s Raw data
   * @return temperature in celsius
   */
  inline double RawToCelsius(const int s) const {
    return B / std::log(R / (s - O) + F) - kT0;
  }
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_PLANCK_H_
