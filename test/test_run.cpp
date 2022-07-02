#include "../include/surgical_tool_manager.h"


int main()
{
    SurgicalToolManager st_manager;

    SurgicalToolParam param(19.990801, 7.8379, 28.012699, 18.5, 5, 0.026835);
    SurgicalToolConfig config(254.099274, -0.284696, 0.003066,
                              -2.981125, 0.169437, -1.879464);

    st_manager.initialize(ENDO, param, SURGICAL_TOOL_TYPE_ENDOSCOPIC, 0);
    printf("params: %s\n", st_manager.getParam(ENDO).info());

    st_manager.updateConfig(ENDO, config);

    auto conf = st_manager.getConfigSpcs(ENDO);
    auto task = st_manager.getTaskSpc(ENDO);
    printf("config size: %d\n", conf.count());
    for(int i = 0; i < conf.count(); i++){
        printf("-- %s\n", conf[i].info());

        printf("\t %s\n", task[i].info());
    }

    mmath::Pose end_pose = st_manager.getEndPose(ENDO);
    printf("end_pose: %s\n", end_pose.info());


    return 0;
}
