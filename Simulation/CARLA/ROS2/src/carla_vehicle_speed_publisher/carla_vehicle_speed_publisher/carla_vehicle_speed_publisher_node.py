import math
import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image


class CarlaVehicleSpeedPublisherNode(Node):
    def __init__(self):
        super().__init__('carla_vehicle_speed_publisher_node')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.image_sub_ = self.create_subscription(Image, '/carla/hero/main_cam/image', self.image_callback, 1)
        self.speed_pub_ = self.create_publisher(Float64, '/vehicle/speed', 1)

        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        self.hero_actor = None
        self._try_find_hero_actor()

        # Retry until the ego vehicle actor is actually spawned/queryable.
        self.actor_search_timer = self.create_timer(0.5, self._retry_find_hero_actor)

    def _try_find_hero_actor(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == 'hero':
                self.hero_actor = actor
                return True
        return False

    def _retry_find_hero_actor(self):
        if self._try_find_hero_actor():
            self.get_logger().info(f'hero_actor found (id={self.hero_actor.id}), ready.')
            self.actor_search_timer.cancel()

    def image_callback(self, msg):
        if self.hero_actor is None:
            self.get_logger().warn('hero_actor not found yet, skipping frame', throttle_duration_sec=2.0)
            return

        velocity = self.hero_actor.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

        out = Float64()
        out.data = speed_kmh
        self.speed_pub_.publish(out)
        self.get_logger().info(f'Velocity published: {speed_kmh:.2f} km/h')


def main(args=None):
    rclpy.init(args=args)
    node = CarlaVehicleSpeedPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()