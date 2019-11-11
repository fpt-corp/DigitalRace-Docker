import rospkg


class Param:
    def __init__(self):
        self.pkg_name = 'dira'
        self.node_name = 'DiRa'
        self.path = rospkg.RosPack().get_path(self.pkg_name)

        self.model_lane_path = self.path + '/models/model-mobilenetv2.json'
        self.model_sign_path = self.path + '/models/sign.h5'
        self.model_lane_weights = self.path + '/models/model-mobilenetv2-round2-snow.h5'
        self.base_speed = 30
        self.min_speed = 20

        self.speed_decay = 2
        self.max_steer_angle = 60.0

        self.steer_angle_scale = 1
        self.middle_pos_scale = 1

        self.min_sign_size = 35

        self.speed_slow_down = 1

        # Delay time
        self.delay_time = 1

        self.turning_time = 0.5
