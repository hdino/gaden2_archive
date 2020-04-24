#ifndef GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED
#define GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED

#include <cmath>
#include <Eigen/Core>

namespace gaden {

static constexpr double PI_POW3 = M_PI * M_PI * M_PI;
static constexpr double SQRT_8_PI_POW3_INV = 1.0 / std::sqrt(8.0 * PI_POW3);

class Filament
{
public:
    Filament(Eigen::Vector3d initial_position,
             double initial_radius,
             double gas_amount_mol)
        : position(initial_position)
        , radius_pow2_(initial_radius * initial_radius)
        , concentration_factor_(gas_amount_mol * SQRT_8_PI_POW3_INV)
    {
        updateRadius();
    }

    inline void addToSquaredRadius(double delta_radius_pow2)
    {
        radius_pow2_ += delta_radius_pow2;
        updateRadius();
    }

    double getConcentrationAt(const Eigen::Vector3d &x) const // returns [mol/m3]
    {
        double distance_pow2 = (x - position).squaredNorm();
        // Neglect influence out of the 3 sigma range
        if (distance_pow2 > radius_pow2_x_9_) return 0;
        return concentration_factor_ * radius_pow3_inv_ * std::exp(-distance_pow2 * radius_pow2_inv_);
    }

    Eigen::Vector3d position; // [m] center of the filament

private:
    inline void updateRadius()
    {
        radius_pow2_x_9_ = 9.0 * radius_pow2_;
        radius_pow2_inv_ = 1.0 / radius_pow2_;
        double radius_inverse = std::sqrt(radius_pow2_inv_);
        radius_pow3_inv_ = radius_pow2_inv_ * radius_inverse;
    }

    double radius_pow2_; // [m2] Controls the size of the filament, R^2(t) in Farrell's paper
                         //      (sigma of a 3D gaussian)
    double radius_pow2_x_9_; // 9*R^2
    double radius_pow2_inv_; // 1/R^2
    double radius_pow3_inv_; // 1/R^3
    double concentration_factor_; // [mol] amount of gas in the filament / sqrt(8
};

} // namespace gaden

#endif // GADEN_SIMULATOR_FILAMENT_HPP_INCLUDED
