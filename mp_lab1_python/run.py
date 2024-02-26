import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR

def main():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    env = Environment(env_idx=1)
    transform = Transform(ur_params)
    
    bb = Building_Blocks(transform=transform, 
                        ur_params=ur_params, 
                        env=env,
                        resolution=0.1, 
                        p_bias=0.05,)
    
    visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)

    # --------- configurations-------------
    
    # home = np.array([0, -1.376, 3, -1.122, 1.570, -2.26 ]) 

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    #                                                                    Q = 2.3
    # home = np.array([-0.694, -1.376, -2.212,  -1.122,  1.570, -2.26]) # do with inflation_factor in {1,3}
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    #                                                                    Q = 3.1
    # collision_free_angles = np.array([0.5, -1.2, 0.8, -0.5, 1.0, 0.7])  
    # home = collision_free_angles
    
    # collision_config_angles = np.array([2.0, 0.0, 9.0, 1.0, 1.0, 3.0]) 
    # home = collision_config_angles
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    #                                                                    Q = 3.2
    # floor_collision_free_angles = np.array([0.5, -1.2, 0.8, -0.5, 1.0, 0.7])  
    # home = floor_collision_free_angles
    
    # wall_collision_config_angles = np.array([0, -1.376, 3, -1.122, 1.570, -2.26 ]) # collides with the wall obstacle!
    # home = wall_collision_config_angles
    
    # floor_collision_config_angles = np.array([0, -1.376, 3, -1.122, 1.570, -2.26 ]) # same configuration collides with the floor!
    # home = floor_collision_config_angles
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
    #                                                                    Q = 4
    conf1 = np.deg2rad([80, -72, 101, -120, -90, -10])
    conf2 = np.deg2rad([20, -90, 90, -90, -90, -10])
    print("local planner return value: " + str(bb.local_planner(conf1, conf2)) )
    num_of_config = max( 3, int( (np.pi / 3) / bb.resolution) )
    print("number of configurations to check in local planner: " + str( num_of_config ) )  # max_angle_diff = np.pi/3 [rad]
    
    home = conf1
    visualizer.show_conf(home)
    
   

if __name__ == '__main__':
    main()



