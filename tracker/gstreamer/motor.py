import pypot.dynamixel as pd
import pypot.robot

config = {
    'gradcap': {
        'controllers': {
            'my_dxl_controller': {
                'sync_read': False,
                'attached_motors':['cap'],
                'port':'auto',
                'baudrate': 1000000,
                'protocol': 2
            }
        },
        'motorgroups': {
            'cap': ['cap_motor']
        },
        'motors': {
            'cap_motor': {
                'orientation': 'direct',
                'type': 'XL-320',
                'id': 1,
                'angle_limit': [-150., 150.],
                'offset': 0.
            }
        }
    }
}

class Motor(object):
    def __init__(self, mirror=False):
        self.motors = pd.Dxl320IO(
            pd.get_available_ports()[0],
            1000000,
        )
        self.motor_id = self.motors.scan(range(20))[0]
        self.motor_id = 1
        self.robot = pypot.robot.from_config(config['gradcap'])
        self.robot.power_up()
        #breakpoint()
        self.motor = self.robot.motors[0]
        self.motor.compliant = False
        #self.motor.goal_speed = 1000.
        self.motor.goto_behavior = 'dummy'
        #breakpoint()
        self.error = [0,0]
        self.position = self.motor.present_position
        self.position_goal = 0
        self.gain = [10., 1.]
        self.move(0)
        self.mirror = mirror

    def update_error(self, error, scale=5.):
        scale *= -1 if self.mirror else 1
        self.error = [self.error[-1], error*scale]

    def calculate_control(self):
        error_p = self.error[-1]
        error_d = self.error[-1]-self.error[0]
        return -(error_p*self.gain[0] + error_d*self.gain[1])

    def move(self, position):
        #self.motors.set_moving_speed({self.motor_id:1000})
        #self.motors.set_goal_position_speed_load({
        #    self.motor_id:[position,1000,100]
        #})
        #self.robot.goto_position({
        #    'cap_motor':position
        #},duration=0.001,wait=False)
        self.motor.compliant = False
        dp = self.motor.goal_position - self.motor.present_position
        #print(position, self.motor.present_position)
        print(self.motor.present_position, self.motor.present_speed)
        self.motor.goto_position(
            position, duration=0.01)
        self.position = self.motor.present_position

    def actuate(self):
        u = self.calculate_control()
        position_goal = self.position + u
        position_goal = min(max(position_goal,-150),150)
        #print(position_goal)
        self.position_goal = (self.position_goal + 0.1)%6
        self.move(position_goal)
