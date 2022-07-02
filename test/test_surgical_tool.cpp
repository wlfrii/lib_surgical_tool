#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "../include/surgical_tool_manager.h"

TEST_CASE("surgical tool parameters", "[SurgicalToolDefine]")
{
    float L1 = 19.990801;
    float Lr = 7.8379;
    float L2 = 28.012699;
    float Lg = 18.5;
    float r = 5;
    float gamma3 = 0.026835;
    SurgicalToolParam param(L1, Lr, L2, Lg, r, gamma3);

    SurgicalToolManager manager;
    manager.initialize(ENDO, param, SURGICAL_TOOL_TYPE_ENDOSCOPIC, 0);
    CHECK(manager.getSurgicalToolType(ENDO) == SURGICAL_TOOL_TYPE_ENDOSCOPIC);

    float L = 254.099274;
    float phi = -0.284696;
    float theta1 = 0.003066;
    float delta1 = -2.981125;
    float theta2 = 0.169437;
    float delta2 = -1.879464;
    SurgicalToolConfig config(L, phi, theta1, delta1, theta2, delta2);
    manager.updateConfig(ENDO, config);
    auto configspcs = manager.getConfigSpcs(ENDO);
    CHECK(configspcs.count() == 4);
    CHECK(configspcs[0].theta == Approx(0).margin(1e-6));
    CHECK(configspcs[0].delta == Approx(phi).margin(1e-6));
    CHECK(configspcs[0].length == Approx(0).margin(1e-6));
    CHECK(configspcs[1].theta == Approx(0).margin(1e-6));
    CHECK(configspcs[1].delta == Approx(0).margin(1e-6));
    CHECK(configspcs[1].length == Approx(L - 225 - L2).margin(1e-6));
    CHECK(configspcs[2].theta == Approx(theta2).margin(1e-6));
    CHECK(configspcs[2].delta == Approx(delta2).margin(1e-6));
    CHECK(configspcs[2].length == Approx(L2).margin(1e-6));
    CHECK(configspcs[3].theta == Approx(0).margin(1e-6));
    CHECK(configspcs[3].delta == Approx(gamma3).margin(1e-6));
    CHECK(configspcs[3].length == Approx(Lg).margin(1e-6));

    auto task = manager.getTaskSpc(ENDO);
    CHECK(configspcs.count() == configspcs.count());

    mmath::Pose end_pose = manager.getEndPose(ENDO);
    CHECK(end_pose.R(0,0) == Approx(0.96430).margin(1e-6));
    CHECK(end_pose.R(0,1) == Approx(0.247452).margin(1e-6));
    CHECK(end_pose.R(0,2) == Approx(-0.094289).margin(1e-6));
    CHECK(end_pose.R(1,0) == Approx(-0.258922).margin(1e-6));
    CHECK(end_pose.R(1,1) == Approx(0.955727).margin(1e-6));
    CHECK(end_pose.R(1,2) == Approx(-0.139803).margin(1e-6));
    CHECK(end_pose.R(2,0) == Approx(0.055520).margin(1e-6));
    CHECK(end_pose.R(2,1) == Approx(0.159226).margin(1e-6));
    CHECK(end_pose.R(2,2) == Approx(0.985680).margin(1e-6));
    CHECK(end_pose.t[0] == Approx(-3.068144).margin(1e-6));
    CHECK(end_pose.t[1] == Approx(1.250816).margin(1e-6));
    CHECK(end_pose.t[2] == Approx(32.200508).margin(1e-6));

}
