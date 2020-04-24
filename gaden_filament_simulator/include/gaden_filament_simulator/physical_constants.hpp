#ifndef GADEN_SIMULATOR_PHYSICAL_CONSTANTS_HPP_INCLUDED
#define GADEN_SIMULATOR_PHYSICAL_CONSTANTS_HPP_INCLUDED

namespace gaden::constant {

// gravity
static constexpr double g = 9.81; // [m/s2]

// gas constant
static constexpr double R = 8.31446261815324; // [m3 Pa / (K * mol)]

// Avogadro constant
static constexpr double N_Avogadro = 6.02214076e23; // [1/mol]

/** Temperature **/
inline constexpr double toKelvinFromDegreeCelsius(double degree_celsius)
{
    return degree_celsius + 273.15;
}

/** Pressure **/
inline constexpr double toPascalFromAtmosphere(double atm)
{
    return atm * 101325.0;
}

inline constexpr double toPascalFromBar(double bar)
{
    return bar * 1e5;
}

//namespace specific_gravity {

//// Specific gravity is the ratio of the density of a substance to the density
//// of a reference substance; equivalently, it is the ratio of the mass of a
//// substance to the mass of a reference substance for the same given volume.
//static constexpr double air = 1.0; // reference

//static constexpr double ethanol  = 1.0378; // heavier than air
//static constexpr double methane  = 0.5537; // lighter than air
//static constexpr double hydrogen = 0.0696; // lighter than air
//static constexpr double acetone  = 1.4529; // heavier than air

////To be updated
//static constexpr double propanol = 58.124; // gases heavier then air
//static constexpr double chlorine = 70.906;
//static constexpr double fluorine = 37.996;
//static constexpr double neon     = 20.179; // gases lighter than air
//static constexpr double helium   =  4.002602;
//static constexpr double hot_air  = 26.966;

//} // namespace specific_gravity

} // namespace gaden::constant

#endif // GADEN_SIMULATOR_PHYSICAL_CONSTANTS_HPP_INCLUDED
