#include <utils.h>
#include <object.h>
#include <shader.h>
#include <bezier.h>
#include <camera.h>

void processInput(GLFWwindow *window);

void mouse_callback(GLFWwindow *window, double x, double y);

const int WIDTH = 1080;
const int HEIGHT = 960;

bool firstMouse = true;
float lastX = WIDTH / 2.0;
float lastY = HEIGHT / 2.0;

GLFWwindow *window;
Camera mycamera;

std::string VS_FILE = "../shader/vs.glsl";
std::string FS_FILE = "../shader/fs.glsl";
std::string BS_FILE = "../bezier/bs1.bsf";
std::string BPT_FILE = "../bezier/teacup.bpt";
std::string BSS_FILE = "../bezier/b-spline.bsp";


enum Render_Case {
    ordinary_single_bezier_surface,
    adaptive_single_bezier_surface,
    ordinary_multi_bezier_surface,
    adaptive_multi_bezier_surface,
    single_spline_surface
};
Render_Case choice = ordinary_single_bezier_surface;


int main() {
    WindowGuard windowGuard(window, WIDTH, HEIGHT, "CS171 hw2");
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, mouse_callback);
    glEnable(GL_DEPTH_TEST);
    // shader
    Shader shader(VS_FILE, FS_FILE);
    shader.use();
    shader.setVec3("objectColor", {1.0f, 1.0f, 1.0f});
    shader.setVec3("lightColor", {1.0f, 0.0f, 0.0f});
    shader.setVec3("lightPos", {6.0f, 6.0f, 6.0f});

    shader.setVec3("viewPos", mycamera.Position);


    Object obj;

    switch (choice) {
        case ordinary_single_bezier_surface: {
            auto bs_ordinary = readBezierSurfaceFromFile(BS_FILE);

            obj = bs_ordinary.generateObject();
            break;
        }
        case adaptive_single_bezier_surface: {
            auto bs_adaptive = readBezierSurfaceFromFile(BS_FILE);
            obj = bs_adaptive.generateObjectAdaptive();
            break;
        }
        case ordinary_multi_bezier_surface: {
            auto mbs_ordinary = readBPT(BPT_FILE);
            obj = generateObject(mbs_ordinary);
            break;
        }
        case adaptive_multi_bezier_surface:{
            auto mbs_adaptive = readBPT(BPT_FILE);
            obj = generateObjectAdaptive(mbs_adaptive);
            break;
        }
        case single_spline_surface:{
            auto ss = readSplineSurfaceFromFile(BSS_FILE);
            obj = ss.generateObject();
            break;
        }
    }

    obj.init();

    // run
    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(mycamera.Zoom), (float) WIDTH / (float) HEIGHT, 0.1f,
                                                100.0f);
        shader.setMat4("projection", projection);
        glm::mat4 view = mycamera.getViewMatrix();
        shader.setMat4("view", view);
        glm::mat4 model = glm::mat4(1.0f);
        shader.setMat4("model", model);


        obj.drawArrays(shader);


        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    return 0;
}

void processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        mycamera.processWalkAround(Forward);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        mycamera.processWalkAround(Backward);
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        mycamera.processWalkAround(Leftward);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        mycamera.processWalkAround(Rightward);
    }
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        std::cout << "Camera position: (" << mycamera.Position.x << ", "
                  << mycamera.Position.y << ", " << mycamera.Position.z << ")" << std::endl;
    }
}

void mouse_callback(GLFWwindow *window, double x, double y) {
    x = (float) x;
    y = (float) y;
    if (firstMouse) {
        lastX = x;
        lastY = y;
        firstMouse = false;
    }

    mycamera.processLookAround(x - lastX, y - lastY);

    // update record
    lastX = x;
    lastY = y;
}
