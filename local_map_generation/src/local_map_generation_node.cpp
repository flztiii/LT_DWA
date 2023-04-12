#include "local_map_generation_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_map_generation_node");
    // 创建代价地图生成类
    std::shared_ptr<LocalMapGeneration> local_map_generator(new LocalMapGeneration());
    ros::spin();
    return 0;
}
