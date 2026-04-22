#include"include/grid_map.hpp"
void init_config(map::GridMapCofig &config) {
    config.global_x_upper_ = 10.0;
    config.global_x_lower_ = -10.0;
    config.global_y_upper_ = 10.0;
    config.global_y_lower_ = -10.0;
    config.voxScale = 0.1;
    config.p_hit = 0.7;
    config.p_miss = 0.4;
    config.p_min = 0.12;
    config.p_max = 0.97;
}
int main(){
    map::GridMapCofig config;
    init_config(config);
    map::GridMap grid_map(config);
    
    return 0;
}