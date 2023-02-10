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

    SurgicalTool tool;
    tool.initialize(param, SURGICAL_TOOL_TYPE_ENDOSCOPIC, 0);
    CHECK(tool.getType() == SURGICAL_TOOL_TYPE_ENDOSCOPIC);

    float L = 254.099274;
    float phi = -0.284696;
    float theta1 = 0.003066;
    float delta1 = -2.981125;
    float theta2 = 0.169437;
    float delta2 = -1.879464;
    SurgicalToolConfig config(L, phi, theta1, delta1, theta2, delta2);
    tool.updateConfig(config);
    auto configspcs = tool.getConfigSpcs();
    REQUIRE(configspcs.size() == 4);
    REQUIRE(configspcs.space_type == ConfigSpaceType::C2);
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

    auto q = configspcs(ConfigSpcs::STEM_BASE);
    CHECK(configspcs[0].theta == Approx(q.theta).margin(1e-6));
    CHECK(configspcs[0].delta == Approx(q.delta).margin(1e-6));
    CHECK(configspcs[0].length == Approx(q.length).margin(1e-6));
    q = configspcs(ConfigSpcs::STEM_RIGID);
    CHECK(configspcs[1].theta == Approx(q.theta).margin(1e-6));
    CHECK(configspcs[1].delta == Approx(q.delta).margin(1e-6));
    CHECK(configspcs[1].length == Approx(q.length).margin(1e-6));
    q = configspcs(ConfigSpcs::STEM_SEG2);
    CHECK(configspcs[2].theta == Approx(q.theta).margin(1e-6));
    CHECK(configspcs[2].delta == Approx(q.delta).margin(1e-6));
    CHECK(configspcs[2].length == Approx(q.length).margin(1e-6));
    q = configspcs(ConfigSpcs::STEM_GRIPPER);
    CHECK(configspcs[3].theta == Approx(q.theta).margin(1e-6));
    CHECK(configspcs[3].delta == Approx(q.delta).margin(1e-6));
    CHECK(configspcs[3].length == Approx(q.length).margin(1e-6));


    auto task = tool.getTaskSpc();
    REQUIRE(configspcs.size() == task.size());
    REQUIRE(configspcs.space_type == task.space_type);

    mmath::Pose end_pose = tool.getEndPose();
    CHECK(end_pose.R(0, 0) == Approx(0.964301).margin(1e-6));
    CHECK(end_pose.R(0, 1) == Approx(0.247452).margin(1e-6));
    CHECK(end_pose.R(0, 2) == Approx(-0.094289).margin(1e-6));
    CHECK(end_pose.R(1, 0) == Approx(-0.258922).margin(1e-6));
    CHECK(end_pose.R(1, 1) == Approx(0.955727).margin(1e-6));
    CHECK(end_pose.R(1, 2) == Approx(-0.139803).margin(1e-6));
    CHECK(end_pose.R(2, 0) == Approx(0.055520).margin(1e-6));
    CHECK(end_pose.R(2, 1) == Approx(0.159226).margin(1e-6));
    CHECK(end_pose.R(2, 2) == Approx(0.985680).margin(1e-6));
    CHECK(end_pose.t[0] == Approx(-3.068144).margin(1e-6));
    CHECK(end_pose.t[1] == Approx(-4.54918).margin(1e-6));
    CHECK(end_pose.t[2] == Approx(47.20051).margin(1e-6));

    auto T = task(TaskSpc::POSE_1B_TO_BASE);
    CHECK(T.R(0, 0) == Approx(0.959747).margin(1e-6));
    CHECK(T.R(0, 1) == Approx(0.280866).margin(1e-6));
    CHECK(T.R(1, 0) == Approx(-T.R(0, 1)).margin(1e-6));
    CHECK(T.R(1, 1) == Approx(T.R(0, 0)).margin(1e-6));
    CHECK(T.t[2] == Approx(0).margin(1e-6));

    T = task(TaskSpc::POSE_2B_TO_1E);
    CHECK(T.R.isIdentity());
    CHECK(T.t[2] == Approx(1.086575).margin(1e-6));

    T = task(TaskSpc::POSE_2E_TO_2B);
    CHECK(T.R(0, 0) == Approx(0.998678).margin(1e-6));
    CHECK(T.R(0, 1) == Approx(-0.004145).margin(1e-6));
    CHECK(T.R(0, 2) == Approx(-0.051227).margin(1e-6));
    CHECK(T.R(1, 0) == Approx(T.R(0, 1)).margin(1e-6));
    CHECK(T.R(1, 1) == Approx(0.987001).margin(1e-6));
    CHECK(T.R(1, 2) == Approx(-0.160658).margin(1e-6));
    CHECK(T.R(2, 0) == Approx(-T.R(0, 2)).margin(1e-6));
    CHECK(T.R(2, 1) == Approx(-T.R(1, 2)).margin(1e-6));
    CHECK(T.R(2, 2) == Approx(0.985690).margin(1e-6));
    CHECK(T.t[0] == Approx(-0.719228).margin(1e-6));
    CHECK(T.t[1] == Approx(-2.255630).margin(1e-6));
    CHECK(T.t[2] == Approx(27.878856).margin(1e-6));

    T = task(TaskSpc::POSE_G_TO_2B);
    CHECK(T.R(0, 0) == Approx(0.998208).margin(1e-6));
    CHECK(T.R(0, 1) == Approx(-0.030940).margin(1e-6));
    CHECK(T.R(0, 2) == Approx(-0.051227).margin(1e-6));
    CHECK(T.R(1, 0) == Approx(0.022340).margin(1e-6));
    CHECK(T.R(1, 1) == Approx(0.986757).margin(1e-6));
    CHECK(T.R(1, 2) == Approx(-0.160658).margin(1e-6));
    CHECK(T.R(2, 0) == Approx(0.055520).margin(1e-6));
    CHECK(T.R(2, 1) == Approx(0.159226).margin(1e-6));
    CHECK(T.R(2, 2) == Approx(0.985690).margin(1e-6));
    CHECK(T.t[0] == Approx(-1.666932).margin(1e-6));
    CHECK(T.t[1] == Approx(-5.227803).margin(1e-6));
    CHECK(T.t[2] == Approx(46.113933).margin(1e-6));
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
    REQUIRE(configspcs.space_type == ConfigSpaceType::C3);
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
    target = manager.getEnd2BasePose(TOOL1);
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


TEST_CASE("surgical tool Jacobian", "[SurgicalToolKine]")
{
    ConfigSpcs qs;
    qs.add(mmath::continuum::ConfigSpc(0, 0.131825, 0));
    qs.add(mmath::continuum::ConfigSpc(0.816344, -1.516124, 25.989237, 1));
    qs.add(mmath::continuum::ConfigSpc(0, 0, 30));
    qs.add(mmath::continuum::ConfigSpc(1.237475, 0.113694, 19.4, 1));
    qs.add(mmath::continuum::ConfigSpc(0, 0, 5.71));
    qs.space_type = ConfigSpaceType::C3;

    Jacobian J = calcJacobian(qs);
    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 6);

    CHECK(J(0, 0) == Approx(39.6991029705163).margin(1e-6));
    CHECK(J(0, 1) == Approx(8.07794109763539).margin(1e-6));
    CHECK(J(0, 2) == Approx(42.6444210769090).margin(1e-6));
    CHECK(J(0, 3) == Approx(0.0715717297876824).margin(1e-6));
    CHECK(J(0, 4) == Approx(6.28886880714537).margin(1e-6));
    CHECK(J(0, 5) == Approx(-2.94531810639269).margin(1e-6));

    CHECK(J(1, 0) == Approx(23.6874472660906).margin(1e-6));
    CHECK(J(1, 1) == Approx(-42.8106381801440).margin(1e-6));
    CHECK(J(1, 2) == Approx(13.1502997843457).margin(1e-6));
    CHECK(J(1, 3) == Approx(-0.379308463732723).margin(1e-6));
    CHECK(J(1, 4) == Approx(10.5964511118459).margin(1e-6));
    CHECK(J(1, 5) == Approx(10.5371474817449).margin(1e-6));

    CHECK(J(2, 0) == Approx(0).margin(1e-6));
    CHECK(J(2, 1) == Approx(-39.9827358880642).margin(1e-6));
    CHECK(J(2, 2) == Approx(-11.5969725477570).margin(1e-6));
    CHECK(J(2, 3) == Approx(0.892573165460230).margin(1e-6));
    CHECK(J(2, 4) == Approx(-8.03085879514480).margin(1e-6));
    CHECK(J(2, 5) == Approx(11.5969725477570).margin(1e-6));

    CHECK(J(3, 0) == Approx(0).margin(1e-6));
    CHECK(J(3, 1) == Approx(0.982659720794466).margin(1e-6));
    CHECK(J(3, 2) == Approx(-0.135104298214221).margin(1e-6));
    CHECK(J(3, 3) == Approx(0).margin(1e-6));
    CHECK(J(3, 4) == Approx(-0.184734378833011).margin(1e-6));
    CHECK(J(3, 5) == Approx(-0.828979088507591).margin(1e-6));

    CHECK(J(4, 0) == Approx(0).margin(1e-6));
    CHECK(J(4, 1) == Approx(0.185418103560956).margin(1e-6));
    CHECK(J(4, 2) == Approx(0.716011810128744).margin(1e-6));
    CHECK(J(4, 3) == Approx(0).margin(1e-6));
    CHECK(J(4, 4) == Approx(0.660904297735112).margin(1e-6));
    CHECK(J(4, 5) == Approx(-0.694166736561761).margin(1e-6));

    CHECK(J(5, 0) == Approx(1).margin(1e-6));
    CHECK(J(5, 1) == Approx(0).margin(1e-6));
    CHECK(J(5, 2) == Approx(0.315110288907621).margin(1e-6));
    CHECK(J(5, 3) == Approx(0).margin(1e-6));
    CHECK(J(5, 4) == Approx(0.727377974998171).margin(1e-6));
    CHECK(J(5, 5) == Approx(0.420189850480678).margin(1e-6));
}
