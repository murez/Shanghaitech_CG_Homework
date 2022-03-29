#include <brdf.h>
#include <utils.h>
#include <interaction.h>

/**
 * IdealDiffusion class
 */

IdealDiffusion::IdealDiffusion(vec3 diff) : reflectivity(diff) {}

vec3 IdealDiffusion::eval(const Interaction &interact) {
    return pdf(interact) * reflectivity;
}

Float IdealDiffusion::pdf(const Interaction &interact) {
    return 1 / (2 * PI);
}


Float IdealDiffusion::sample(Interaction &interact) {
    float e1 = unif(0, 1, 1)[0];
    float e2 = unif(0, 1, 1)[0];
    float r = sqrt(e1);
    float theta = 2 * PI * e2;
    float x = r * cosf(theta);
    float y = r * sinf(theta);
    float z = sqrt(1 - e1);

    vec3 local_wi(x, y, z);
    Eigen::Matrix3f rotate = Eigen::Matrix3f::Zero();
    rotate = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1.0f), interact.normal).toRotationMatrix();

    interact.wi = rotate * local_wi;

    float pdf = sqrt(1 - e1) / PI;
    return pdf;
}

bool IdealDiffusion::isDelta() const { return false; }

std::shared_ptr<BRDF> makeIdealDiffusion(const vec3 &diff) {
    return std::make_shared<IdealDiffusion>(diff);
}



IdealSpecular::IdealSpecular(vec3 spec) : reflectivity(spec) {}

vec3 IdealSpecular::eval(const Interaction &interact) {
    return pdf(interact) * reflectivity;
}

float IdealSpecular::pdf(const Interaction &interact) {
    vec3 ref = 2 * interact.normal.dot(interact.wo) * interact.normal - interact.wo;
    if ((ref - interact.wi).norm() < 1e-3) return 1.0f;
    return 0.0f;
}

float IdealSpecular::sample(Interaction &interact) {
    interact.wi = 2 * interact.normal.dot(interact.wo) * interact.normal - interact.wo;
    return 1.0f;
}

bool IdealSpecular::isDelta() const {
    return true;
}

std::shared_ptr<BRDF> makeIdealSpecular(const vec3 &spec) {
    return std::make_shared<IdealSpecular>(spec);
}