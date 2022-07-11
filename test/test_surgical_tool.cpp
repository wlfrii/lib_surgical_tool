#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <surgical_tool_manager.h>

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
    CHECK(manager.getType(ENDO) == SURGICAL_TOOL_TYPE_ENDOSCOPIC);

    float L = 254.099274;
    float phi = -0.284696;
    float theta1 = 0.003066;
    float delta1 = -2.981125;
    float theta2 = 0.169437;
    float delta2 = -1.879464;
    SurgicalToolConfig config(L, phi, theta1, delta1, theta2, delta2);
    manager.updateConfig(ENDO, config);
    auto configspcs = manager.getConfigSpcs(ENDO);
    REQUIRE(configspcs.size() == 4);
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
    REQUIRE(configspcs.size() == configspcs.size());

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


TEST_CASE("surgical tool kinematics", "[SurgicalToolKine]")
{
    SurgicalToolParam tool_param(79.2, 30, 19.4, 5.71, 3.5);
    float L = 17.989237 + tool_param.getL2() + tool_param.getLr();
    float phi = 0.211825;
    float theta1 = 0.706244;
    float delta1 = -2.018124;
    float theta2 = 0.912775;
    float delta2 = 1.011369;
    SurgicalToolConfig tool_config(L, phi, theta1, delta1, theta2, delta2);
    SurgicalToolManager manager;
    manager.initialize(TOOL1, tool_param, SURGICAL_TOOL_TYPE_SP_TOOL, 0);
    manager.updateConfig(TOOL1, tool_config);
    auto base_pose = manager.getBasePose(TOOL1);
    CHECK(base_pose.t[1] == Approx(-7.85).margin(1e-6));

    auto configspcs = manager.getConfigSpcs(TOOL1);
    REQUIRE(configspcs.size() == 5);
    CHECK(configspcs[0].theta == Approx(0).margin(1e-6));
    CHECK(configspcs[0].delta == Approx(phi).margin(1e-6));
    CHECK(configspcs[0].length == Approx(0).margin(1e-6));
    CHECK(configspcs[1].theta == Approx(theta1).margin(1e-6));
    CHECK(configspcs[1].delta == Approx(delta1).margin(1e-6));
    CHECK(configspcs[1].length == Approx(L - tool_param.getL2() -
                                         tool_param.getLr()).margin(1e-6));
    CHECK(configspcs[2].theta == Approx(0).margin(1e-6));
    CHECK(configspcs[2].delta == Approx(0).margin(1e-6));
    CHECK(configspcs[2].length == Approx(tool_param.getLr()).margin(1e-6));
    CHECK(configspcs[3].theta == Approx(theta2).margin(1e-6));
    CHECK(configspcs[3].delta == Approx(delta2).margin(1e-6));
    CHECK(configspcs[3].length == Approx(tool_param.getL2()).margin(1e-6));
    CHECK(configspcs[4].theta == Approx(0).margin(1e-6));
    CHECK(configspcs[4].delta == Approx(0).margin(1e-6));
    CHECK(configspcs[4].length == Approx(tool_param.getLg()).margin(1e-6));

    auto end_pose = manager.getEndPose(TOOL1);
    CHECK(end_pose.R(0,0) == Approx(0.958300131788949).margin(1e-6));
    CHECK(end_pose.R(0,1) == Approx(-0.252909872726839).margin(1e-6));
    CHECK(end_pose.R(0,2) == Approx(0.133031776995487).margin(1e-6));
    CHECK(end_pose.R(1,0) == Approx(0.228442897054108).margin(1e-6));
    CHECK(end_pose.R(1,1) == Approx(0.957685179182800).margin(1e-6));
    CHECK(end_pose.R(1,2) == Approx(0.175079811397928).margin(1e-6));
    CHECK(end_pose.R(2,0) == Approx(-0.171681974006618).margin(1e-6));
    CHECK(end_pose.R(2,1) == Approx(-0.137388841799114).margin(1e-6));
    CHECK(end_pose.R(2,2) == Approx(0.975525297442505).margin(1e-6));
    CHECK(end_pose.t[0] == Approx(-5.396805608642865).margin(1e-6));
    CHECK(end_pose.t[1] == Approx(-36.464950740329022).margin(1e-6));
    CHECK(end_pose.t[2] == Approx(63.043203955522713).margin(1e-6));

    mmath::Pose target = base_pose.inverse() * end_pose;
    SurgicalToolConfig config_ik;
    bool flag = calcInverseKinematicsC3(target, tool_param, config_ik);
    REQUIRE(flag == true);
    CHECK(config_ik.L_insert + 30 + 19.4 == Approx(L).margin(1e-6));
    CHECK(config_ik.phi == Approx(phi).margin(1e-6));
    CHECK(config_ik.theta1 == Approx(theta1).margin(1e-6));
    CHECK(config_ik.delta1 == Approx(delta1).margin(1e-6));
    CHECK(config_ik.theta2 == Approx(theta2).margin(1e-6));
    CHECK(config_ik.delta2 == Approx(delta2).margin(1e-6));

    L = 25.989237 + tool_param.getL2() + tool_param.getLr();
    phi = 0.131825;
    theta1 = 0.816344;
    delta1 = -1.516124;
    theta2 = 1.237475;
    delta2 = 0.113694;
    tool_config = SurgicalToolConfig(L, phi, theta1, delta1, theta2, delta2);
    manager.updateConfig(TOOL1, tool_config);
    end_pose = manager.getEndPose(TOOL1);
    target = base_pose.inverse() * end_pose;
    CHECK(target.R(0,0) == Approx(0.210556828377472).margin(1e-6));
    CHECK(target.R(0,1) == Approx(-0.161892079703181).margin(1e-6));
    CHECK(target.R(0,2) == Approx(0.964083386721812).margin(1e-6));
    CHECK(target.R(1,0) == Approx(0.670329975244716).margin(1e-6));
    CHECK(target.R(1,1) == Approx(0.741741543294745).margin(1e-6));
    CHECK(target.R(1,2) == Approx(-0.021845073566982).margin(1e-6));
    CHECK(target.R(2,0) == Approx(-0.711564154740834).margin(1e-6));
    CHECK(target.R(2,1) == Approx(0.650853622161011).margin(1e-6));
    CHECK(target.R(2,2) == Approx(0.264699860611702).margin(1e-6));
    CHECK(target.t[0] == Approx(23.687447266090576).margin(1e-6));
    CHECK(target.t[1] == Approx(-39.699102970516300).margin(1e-6));
    CHECK(target.t[2] == Approx(55.854892705416283).margin(1e-6));

    flag = calcInverseKinematicsC3(target, tool_param, config_ik);
    REQUIRE(flag == true);
    CHECK(config_ik.L_insert + 30 + 19.4 == Approx(L).margin(1e-6));
    CHECK(config_ik.phi == Approx(phi).margin(1e-6));
    CHECK(config_ik.theta1 == Approx(theta1).margin(1e-6));
    CHECK(config_ik.delta1 == Approx(delta1).margin(1e-6));
    CHECK(config_ik.theta2 == Approx(theta2).margin(1e-6));
    CHECK(config_ik.delta2 == Approx(delta2).margin(1e-6));
}
