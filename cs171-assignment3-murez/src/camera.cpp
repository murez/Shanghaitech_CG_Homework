#include <camera.h>
#include <ray.h>

Camera::Camera(const vec3 &pos, Float vFov, const vec2i &resolution,
               Float focalLen)
    : film(resolution),
      verticalFov(vFov),
      position(pos),
      focalLength(focalLen),
      forward(0, 0, 1),
      right(1, 0, 0),
      up(0, 1, 0) {}

/**
 * do camera look at
 * @param[in] lookAt look-at point
 * @param[in] refUp reference up vector
 */
void Camera::lookAt(const vec3 &lookAt, const vec3 &refUp) {
  forward = -(lookAt - position).normalized();
  right = refUp.cross(forward).normalized();
  up = forward.cross(right).normalized();
  // DELETE THIS WHEN FINISHED
  //  UNIMPLEMENTED;
}

/**
 * generate a ray in the world space with given film sample (dx, dy)
 * @param[in] dx x in the film
 * @param[in] dy y in the film
 */
Ray Camera::generateRay(Float dx, Float dy) const {

  float imageAspectRatio = film.getAspectRatio(); // assuming width > height
  float Px = dx * tan(verticalFov / 2 * M_PI / 180) * imageAspectRatio;
  float Py = dy * tan(verticalFov / 2 * M_PI / 180);

  Eigen::Matrix<float, 3, 3> transformation;
  transformation.col(0) << right.x(), right.y(), right.z();
  transformation.col(1) << up.x(), up.y(), up.z();
  transformation.col(2) << forward.x(), forward.y(), forward.z();
  vec3 ori(dx, dy, -focalLength);
  vec3 r_pos = transformation * ori + position;
  vec3 r_dir = transformation * ori;
  return Ray(r_pos, r_dir);

  // vec3 dir;
  // TODO: generate camera ray
  // !!!DELETE THIS WHEN FINISHED
  //  UNIMPLEMENTED;
  //  return Ray{position, dir};
}

void Camera::setPixel(int dx, int dy, const vec3 &value) {
  film.pixels[dy * film.resolution.x() + dx] = value;
}

const Film &Camera::getFilm() const { return film; }

std::shared_ptr<Camera> makeCamera(const vec3 &pos, Float vFov,
                                   const vec2i &resolution, Float focalLen) {
  return std::make_shared<Camera>(pos, vFov, resolution, focalLen);
}