#ifndef __MATH_UTILS
#define __MATH_UTILS

#include <cmath>

const float epsilon = 0.01f;

inline bool compare_floats(float x, float y) {
    if(fabs(x-y) < epsilon) {
        return true;
    }

    return false;
}

inline bool compare_int_and_float(int x, float y) {
    float aux = x;

    if(fabs(aux-y) < epsilon) {
        return true;
    }

    return false;
}

inline bool greater_than_floats(float x, float y) {
    if((x-y) >= epsilon) {
        return true;
    }

    return false;
}

inline bool greater_than_int_and_float(int x, float y) {
    float aux = x;

    if((aux-y) >= epsilon) {
        return true;
    }

    return false;
}

inline bool greater_than_float_and_int(int x, float y) {
    float aux = x;

    if((y-aux) >= epsilon) {
        return true;
    }

    return false;
}

#endif