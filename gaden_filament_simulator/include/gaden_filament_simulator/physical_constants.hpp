#ifndef GADEN_SIMULATOR_PHYSICAL_CONSTANTS_HPP_INCLUDED
#define GADEN_SIMULATOR_PHYSICAL_CONSTANTS_HPP_INCLUDED

namespace gaden::constant {

// gas constant
static constexpr double R = 8.31446261815324; // [m3 Pa / (K * mol)]

static constexpr double g = 9.81; // [m/s2]

double density_air = 1.205;           // [kg/m3] density of air
double dynamic_viscosity_air = 19e-6; // [kg/(m*s)] dynamic viscosity of air, [Pa s]

namespace specific_gravity {

// Specific gravity is the ratio of the density of a substance to the density
// of a reference substance; equivalently, it is the ratio of the mass of a
// substance to the mass of a reference substance for the same given volume.
static constexpr double air = 1.0; // reference

static constexpr double ethanol  = 1.0378; // heavier than air
static constexpr double methane  = 0.5537; // lighter than air
static constexpr double hydrogen = 0.0696; // lighter than air
static constexpr double acetone  = 1.4529; // heavier than air

//To be updated
static constexpr double propanol = 58.124; // gases heavier then air
static constexpr double chlorine = 70.906;
static constexpr double fluorine = 37.996;
static constexpr double neon     = 20.179; // gases lighter than air
static constexpr double helium   =  4.002602;
static constexpr double hot_air  = 26.966;

} // namespace specific_gravity

} // namespace gaden::constant

#endif // GADEN_SIMULATOR_PHYSICAL_CONSTANTS_HPP_INCLUDED
