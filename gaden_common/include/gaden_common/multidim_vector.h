#ifndef GADEN_COMMON_MULTIDIM_VECTOR_H_INCLUDED
#define GADEN_COMMON_MULTIDIM_VECTOR_H_INCLUDED

#include <vector>

namespace gaden {

template <typename T>
class Vector2D : public std::vector<std::vector<T>>
{
public:
    Vector2D(size_t dim0, size_t dim1, T init_value = T())
    {
        this->resize(dim0, std::vector<T>(dim1, init_value));
    }
};

template <typename T>
class Vector3D : public std::vector<std::vector<std::vector<T>>>
{
    //
};

}

#endif // GADEN_COMMON_MULTIDIM_VECTOR_H_INCLUDED
