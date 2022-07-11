#include <surgical_tool_manager.h>
#include <layer_cylinder.h>
#include <layer_segment.h>
#include <layer_gripper.h>
#include <gl_util.h>
#include <vector>
#include <iostream>

void keyboardControlModel(GLFWwindow* window);

int height = 1080;
int width = 1920;
glm::mat4 model = glm::mat4(-0.290462,-0.251291,0.923312,0.000000,
                            -0.620875,-0.684736,-0.381675,0.000000,
                            0.728125,-0.684110,0.042866,0.000000,
                            -41.451847,16.748739,106.138512,1.000000);

float theta1 = 0;
float theta2 = 0;

std::vector<Layer*>& addLayer(std::vector<Layer*>& layers, const ConfigSpcs &configspcs, const TaskSpc &taskspc, float radius, SurgicalToolType type);

int main()
{
    gl_util::Window window(width, height);
    window.enableDepthTest();
    window.setKeyboardEventCallBack(keyboardControlModel);

    // Init Surgical Tool
    float L1 = 19.990801;
    float Lr = 7.8379;
    float L2 = 28.012699;
    float Lg = 18.5;
    float radius = 5;
    float gamma3 = 0.026835;
    SurgicalToolParam endo_param(L1, Lr, L2, Lg, radius, gamma3);
    float L = 254.099274;
    float phi = -0.284696;
    theta1 = 0.003066;
    float delta1 = -2.981125;
    theta2 = 0.169437;
    float delta2 = -1.879464;
    SurgicalToolConfig config(L, phi, theta1, delta1, theta2, delta2);

    SurgicalToolManager manager;
    manager.initialize(ENDO, endo_param, SURGICAL_TOOL_TYPE_ENDOSCOPIC, 0);
    manager.updateConfig(ENDO, config);

    L1 = 79.2;
    Lr = 30;
    L2 = 19.4;
    Lg = 5.71;
    radius = 4;
    SurgicalToolParam tool_param(L1, Lr, L2, Lg, radius);
    manager.initialize(TOOL1, tool_param, SURGICAL_TOOL_TYPE_SP_TOOL, 0);
    L = 17.989237 + tool_param.getL2() + tool_param.getLr();
    phi = 0.211825;
    theta1 = 0.706244;
    delta1 = -2.018124;
    theta2 = 0.912775;
    delta2 = 1.011369;
    config = SurgicalToolConfig(L, phi, theta1, delta1, theta2, delta2);
    manager.updateConfig(TOOL1, config);

    // Init Render property
    glm::mat4 view = glm::rotate(glm::mat4(1.0), glm::radians(180.f),
                                 glm::vec3(1.f,0.f,0.f));
    view[3][0] += 2.f;
    LayerModel::setView(view, 0);
    view[3][0] -= 4.f;
    LayerModel::setView(view, 1);
    gl_util::Projection gl_proj(1120, 960, 540, width, height, 0.2, 150);
    LayerModel::setProjection(gl_proj.mat4());

    std::vector<Layer*> endo_layers, tool_layers;
    // Build endoscope
    auto configspcs_e = manager.getConfigSpcs(ENDO);
    auto taskspc_e = manager.getTaskSpc(ENDO);
    endo_layers = addLayer(endo_layers, configspcs_e, taskspc_e, radius,
                      manager.getType(ENDO));
    auto endo_base = manager.getBasePose(ENDO);

    auto configspcs_t = manager.getConfigSpcs(TOOL1);
    auto taskspc_t = manager.getTaskSpc(TOOL1);
    tool_layers = addLayer(tool_layers, configspcs_t, taskspc_t, radius,
                      manager.getType(TOOL1));

    while (!window.shouldClose()) {
        window.activate();
        window.clear();
        glClearDepth(1.0);

        glm::mat4 model_base = model*cvt2GlmMat4(endo_base);
        for(int i = 0; i < configspcs_e.size(); i++){
            auto layer = dynamic_cast<LayerModel*>(endo_layers[i]);
            layer->setModel(model_base);
            layer->render(LAYER_RENDER_LEFT);
            model_base *= cvt2GlmMat4(taskspc_e[i]);
        }

        LayerSegment* layer_seg = dynamic_cast<LayerSegment*>(tool_layers[1]);
        //if (theta1 < 0) delta1 = delta1 + M_PI;
        layer_seg->setProperty({17.989237, theta1, delta1, radius});
        taskspc_t[1] = mmath::continuum::calcSingleSegmentPose(17.989237, theta1, delta1);
//        layer_seg = dynamic_cast<LayerSegment*>(tool_layers[3]);
//        layer_seg->setProperty({17.989237, theta1, delta1, radius});

        auto tool_base = manager.getBasePose(TOOL1);
        model_base = model*cvt2GlmMat4(tool_base);
        for(int i = 0; i < configspcs_t.size(); i++){
            auto layer = dynamic_cast<LayerModel*>(tool_layers[i]);
            layer->setModel(model_base);
            layer->render(LAYER_RENDER_LEFT);
            model_base *= cvt2GlmMat4(taskspc_t[i]);
        }

        window.refresh();
    }
    window.release();

    // Release layer
    for(auto& layer : endo_layers){
        if(layer){
            delete layer;
            layer = nullptr;
        }
    }
    for(auto& layer : tool_layers){
        if(layer){
            delete layer;
            layer = nullptr;
        }
    }

    return 0;
}


std::vector<Layer*>& addLayer(std::vector<Layer*>& layers,
                              const ConfigSpcs & configspcs,
                              const TaskSpc &taskspc, float radius,
                              SurgicalToolType type)
{
    glm::mat4 model_base = model;
    for(int i = 0; i < configspcs.size(); i++){
        auto& q = configspcs[i];
        float len = std::max(q.length, 0.01f);

        glm::vec3 color = glm::vec3(1.0, 1.0, 0.0);
        if(i == configspcs.size() - 1){
            color = glm::vec3(1.0f, 1.0f, 1.0f);
        }

        LayerModel* layer_obj;
        if(q.is_bend){
            layer_obj = new LayerSegment(width, height, LAYER_RENDER_2D, color, {len, q.theta, q.delta, radius});
        }
        else{
            if(i == configspcs.size() - 1 &&
                    type == SURGICAL_TOOL_TYPE_SP_TOOL){
                layer_obj = new LayerGripper(width, height, LAYER_RENDER_2D, color, GRIPPER_NEEDLE_HOLDER);
            }
            else{
                layer_obj = new LayerCylinder(width, height, LAYER_RENDER_2D, color, {glm::vec3(0.f, 0.f, 0.f), len, radius});
            }
        }
        layer_obj->setModel(model_base);
        layers.emplace_back(layer_obj);

        model_base *= cvt2GlmMat4(taskspc[i]);
    }

    return layers;
}


void keyboardControlModel(GLFWwindow* window)
{
    float step = 0.5;
    float r_step = 3.f;

    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS){
        glfwSetWindowShouldClose(window, true);
    }
    else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, step, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, -step, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(-step, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(step, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, 0.f, step));
    }
    else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS){
        model = glm::translate(model, glm::vec3(0.f, 0.f, -step));
    }
    else if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(-r_step), glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(r_step), glm::vec3(1.f, 0.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(-r_step), glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(r_step), glm::vec3(0.f, 1.f, 0.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(-r_step), glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS){
        model = glm::rotate(model, glm::radians(r_step), glm::vec3(0.f, 0.f, 1.f));
    }
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS){
        gl_util::print("Model", model);
    }
    else if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS){
        theta1 += 0.1;
    }
    else if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS){
        theta1 -= 0.1;
    }
    else if (glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS){
        theta2 += 0.1;
    }
    else if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS){
        theta2 -= 0.1;
    }
    theta1 = std::fminf(std::fmaxf(theta1, -M_PI/2.f), M_PI/2.f);
    theta2 = std::fminf(std::fmaxf(theta1, -M_PI/2.f), M_PI/2.f);
}
