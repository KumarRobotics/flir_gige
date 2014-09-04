#ifndef FLIR_GIGE_PLANCK_H_
#define FLIR_GIGE_PLANCK_H_

#include <cmath>

namespace flir_gige {

/**
 * @brief The Planck constants from flir thermal camera
 */
struct Planck {
  Planck() : B{0}, F{0}, O{0}, R{0} {}
  Planck(double B, double F, double O, double R) : B{B}, F{F}, O{O}, R{R} {}

  double B;
  double F;
  double O;
  double R;
  static constexpr double kT0{273.15};  ///< Kelvin at 0 celcius

  /**
   * @brief CelsiusToRaw Convert celsius to 16-bit raw data
   * @param t Celcius
   * @return raw data
   */
  int CelsiusToRaw(const double t) const {
    return R / (std::exp(B / (t + kT0)) - F) + O;
  }

  /**
   * @brief RawToCelsius Convert 16-bit raw data to celsius
   * @param s Raw data
   * @return temperature in celsius
   */
  double RawToCelsius(const int s) const {
    return B / std::log(R / (s - O) + F) - kT0;
  }
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_PLANCK_H_
