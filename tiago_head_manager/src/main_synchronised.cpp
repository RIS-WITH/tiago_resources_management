#include "tiago_head_manager/tiago_head_manager.h"

int main(int argc, char *argv[]){
    ros::init(argc,argv,"tiago_head_manager");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    TiagoHeadManager mgr(nh, plugins, true);

    std::thread th(&TiagoHeadManager::run, &mgr);

    ros::spin();

    th.join();

    return 0;
}
