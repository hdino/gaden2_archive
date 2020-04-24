#ifndef GADEN_COMMON_CHEMICAL_SUBSTANCE_HPP_INCLUDED
#define GADEN_COMMON_CHEMICAL_SUBSTANCE_HPP_INCLUDED

namespace gaden::chemicals {

class ChemicalBase
{
public:
    virtual double getMassDensity() const = 0; // [kg/m3]
    virtual double getMolarMass() const = 0; // [kg/mol]
};

class Air : public ChemicalBase
{
public:
    static double DynamicViscosity() { return 18.2e-6; } // [kg/(m*s)] = [Pa s], at 0° C, 1013 hPa
    static double MassDensity() { return 1.2920; } // [kg/m3], at 0° C, 1013 hPa
    static double MolarMass() { return 28.9e-3; } // [kg/mol]

    double getMassDensity() const { return MassDensity(); }
    double getMolarMass() const { return MolarMass(); }
};

class Methane : public ChemicalBase
{
public:
    double getMassDensity() const { return 0.72; } // [kg/m3], at 0° C, 1013 hPa
    double getMolarMass() const { return 16.043e-3; } // [kg/mol]
};

} // namespace gaden::chemicals

#endif // GADEN_COMMON_CHEMICAL_SUBSTANCE_HPP_INCLUDED
