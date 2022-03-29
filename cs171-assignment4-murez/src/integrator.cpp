#include <integrator.h>
#include <brdf.h>
#include <light.h>
#include <utils.h>
#include <iomanip>

#define DELTA 0.0001f
#define EPSILON 0.00001f

/**
 * Integrator class
 */
Integrator::Integrator(std::shared_ptr<Camera> camera) : camera(camera) {}

/**
 * PhongLightingIntegrator class
 */
//PathIntegrator::PathIntegrator(std::shared_ptr<Camera> camera)
//        : Integrator(camera) {}
PathIntegrator::PathIntegrator(std::shared_ptr<Camera> camera, PathIntegrator::SampleType sample_type_)
        : Integrator(camera),
          sampleType(sample_type_) {
    if (sampleType == PathIntegrator::SampleType::HYBRID)
        printf("sample type is hybrid.\n");
    if (sampleType == PathIntegrator::SampleType::LIGHT)
        printf("sample type is light.\n");
    if (sampleType == PathIntegrator::SampleType::BRDF)
        printf("sample type is brdf.\n");
}

/**
 * render a scene
 * @param[in] the given scene
 */
void PathIntegrator::render(Scene &scene) {

int now=0;
    int res_x = camera->getFilm().resolution.x(), res_y = camera->getFilm().resolution.y();
    vec2 rotate_col1 = Eigen::Vector2f(2, -1).normalized();
    vec2 rotate_col2 = Eigen::Vector2f(1, 2).normalized();

    Eigen::Matrix<float, 2, 2> rotate;
    rotate.col(0) << rotate_col1.x(), rotate_col1.y();
    rotate.col(1) << rotate_col2.x(), rotate_col2.y();

    int spp = 16;
    int sp = sqrt(spp);
    float delta_x = 1.0f / sp;
    float delta_y = 1.0f / sp;

    float step = 0;
#ifndef NO_OMP
    omp_set_num_threads(128);
#pragma omp parallel for schedule(guided, 2) default(none) shared(now, res_x, res_y, delta_x, delta_y, step, rotate, spp, sp, scene)
#endif
    for (int dx = 0; dx < res_x; ++dx) {
#ifdef USE_OPENMP
#pragma omp atomic
#endif
        ++now;
        printf("\r%.02f%%", (float) now * 100.0 / res_x);
        for (int dy = 0; dy < res_y; ++dy) {

            vec3f px_rad = vec3f::Zero();

            for (int i = 0; i < sp; i++) {
                for (int j = 0; j < sp; j++) {
                    vec2f shift(delta_x * i - 0.5, delta_y * j - 0.5);
                    shift = rotate * shift;
                    float x = dx + 0.5 + shift.x();
                    float y = dy + 0.5 + shift.y();
                    Ray ray = camera->generateRay(x, y);
                    px_rad += radiance(scene, ray);
                }
            }

            px_rad /= (float) sp * sp;
            camera->setPixel(dx, dy, px_rad);
        }
    }
    printf("\n");
}


vec3 ld(Scene &scene, Interaction& interaction) {
    vec3f dir_light = vec3f::Zero();
    vec3f dir_brdf = vec3f::Zero();
    float pdf_light = 1.0f;
    float pdf_brdf = 1.0f;
    vec3f sample_pos = scene.getLight()->sample(interaction, &pdf_light);
    Ray new_ray(interaction.entryPoint + 0.01 * interaction.wi, 1 * interaction.wi);
    if (!scene.isShadowed(new_ray)) {
        float ex_term = scene.getLight()->pdf(interaction, sample_pos);
        vec3f L_i = scene.getLight()->emission(sample_pos, -interaction.wi);

        vec3f f_r = interaction.brdf->eval(interaction);
        dir_light =
                interaction.normal.dot(interaction.wi) * ex_term * (L_i.array() * f_r.array()).matrix() / pdf_light;
    }
    pdf_brdf = interaction.brdf->sample(interaction);
    Ray next_path_brdf(interaction.entryPoint + 0.001 * interaction.wi, 1 * interaction.wi);
    if (!scene.isShadowed(next_path_brdf)) {
        vec3f f_r = interaction.brdf->eval(interaction);
        vec3f L_i = scene.getLight()->emission(sample_pos, -interaction.wi);
        dir_brdf = interaction.normal.dot(interaction.wi) * (L_i.array() * f_r.array()).matrix() / pdf_brdf;
    }
    dir_light = (pdf_light * pdf_light / (pdf_brdf * pdf_brdf + pdf_light * pdf_light)) * dir_light +
                (pdf_brdf * pdf_brdf / (pdf_brdf * pdf_brdf + pdf_light * pdf_light)) * dir_brdf;
    return dir_light;
}

/**
 * calculate the radiance with given scene, ray and interaction
 * @param[in] scene the given scene
 * @param[in] interaction the given interaction
 * @param[in] the given ray
 */
vec3f PathIntegrator::radiance(Scene &scene, const Ray &ray) {
    Eigen::Vector3f beta(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f L(0.0f, 0.0f, 0.0f);
    Ray r = ray;
    for (int i = 0; i < 5; ++i)
    {
        Interaction interaction;
        scene.intersect(r, interaction);
        // The pixel beam directly hits the light source
        if (interaction.type == Interaction::Type::LIGHT && i == 0)
        {
            L += scene.getLight()->emission(interaction.entryPoint, -r.direction);
            break;
        }

        if (interaction.type == Interaction::Type::NONE)
        {
            break;
        }

        if (interaction.type == Interaction::Type::LIGHT)
        {
            continue;
        }

        L += beta.cwiseProduct(ld(scene, interaction));

        // Sample wo according to brdf
        auto brdf = interaction.brdf;
        float pdf = brdf->sample(interaction);

        // update beta
        float cos = abs(interaction.normal.dot(interaction.wo)/interaction.normal.norm()/interaction.wo.norm());
        vec3f update = brdf->eval(interaction) * cos / pdf;
        beta = beta.cwiseProduct(update);

        // next ray
        r = Ray(interaction.entryPoint, interaction.wo);
    }

    return L;
}



std::shared_ptr<Integrator> makePathIntegrator(std::shared_ptr<Camera> camera) {
    return std::make_shared<PathIntegrator>(camera, PathIntegrator::SampleType::HYBRID);
}
