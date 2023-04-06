#ifndef LIB_SURGICAL_TOOL_ARRAY_REPO_H_LF
#define LIB_SURGICAL_TOOL_ARRAY_REPO_H_LF
#include <stdint.h>
#include <initializer_list>

/**
 * @brief A template class store some objects with same type T.
 * The object can be added by add() interface and accessed with 
 * operator[] interface. The size can be get by count() interface.
 * 
 * @tparam T Typename.
 * @tparam N The number of the objects.
 */
template <typename T, uint8_t N>
class ArrayRepo
{
public:
    ArrayRepo()
        : _count(0)
    {}

    bool add(const T& item)
    {
        if (_count >= N){
            return false;
        }
        _elements[_count++] = item;
        return true;
    }

    const T& operator[](uint8_t index) const
    {
        return _elements[index];
    }

    T& operator[](uint8_t index)
    {
        return _elements[index];
    }

    int size() const
    {
        return _count;
    }

protected:
    uint8_t _count;
    T	 	_elements[N];
};


#endif // LIB_SURGICAL_TOOL_ARRAY_REPO_H_LF
