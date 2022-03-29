#ifndef CS171_HW4_INCLUDE_UTILS_H_
#define CS171_HW4_INCLUDE_UTILS_H_

#include <core.h>
#include <random>

inline std::vector<Float> unif(Float a, Float b, int N) {
    static std::default_random_engine engine;
    std::vector<Float> res;
    std::uniform_real_distribution<Float> dis(a, b);

    res.reserve(N);
    for (int i = 0; i < N; i++) {
        res.push_back(dis(engine));
    }
    return res;
}

inline float vec_cos(vec3 a, vec3 b) {
    return abs(a.dot(b) / a.norm() / b.norm());
}

#endif  // CS171_HW4_INCLUDE_UTILS_H_
