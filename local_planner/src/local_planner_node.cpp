#include "local_planner_node.hpp"

int main(int argc, char** argv) {
    // 初始化ros
    ros::init(argc, argv, "local_planner_node");
    // 构建节点
    std::unique_ptr<LocalPlannerNode> local_planner_node_ptr = std::make_unique<LocalPlannerNode>();
    // 进行规划
    getchar();
    int count = 0;
    if (local_planner_node_ptr->getSimulationName() == "orca") {
        for (int i = 0; i < 300; i++) {
            int result = local_planner_node_ptr->planOrcaOnce(i);
            if (result > 0) {
                count++;
            }
            std::cout << "test times: " << i + 1 << ", success times: " << count << std::endl;
        }
    } else if (local_planner_node_ptr->getSimulationName() == "crowd"){
        for (int i = 0; i < 300; i++) {
            int result = local_planner_node_ptr->planCrowdOnce(i);
            if (result > 0) {
                count++;
            }
            std::cout << "test times: " << i + 1 << ", success times: " << count << std::endl;
        }
    } else if (local_planner_node_ptr->getSimulationName() == "static") {
        for (int i = 0; i < 300; i++) {
            int result = local_planner_node_ptr->planStaticOnce(i);
            if (result > 0) {
                count++;
            }
            std::cout << "test times: " << i + 1 << ", success times: " << count << std::endl;
        }
    } else {
        throw;
    }
    std::cout << "final success count: " << count << std::endl;
    return 0;
}