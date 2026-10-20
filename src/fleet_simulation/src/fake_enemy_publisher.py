#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from fleet_interfaces.msg import Enemy, EnemyArray


class FakeEnemyPublisher(Node):

    def __init__(self):
        super().__init__('fake_enemy_publisher')

        self.publisher = self.create_publisher(
            EnemyArray,
            '/terrascout1/enemies',
            10
        )

        self.counter = 0
        self.timer = self.create_timer(1.0, self.publish_enemies)

    def publish_enemies(self):
        msg = EnemyArray()

        enemy1 = Enemy()
        enemy1.id = 1
        enemy1.x = 4.0
        enemy1.y = 2.0

        enemy2 = Enemy()
        enemy2.id = 7
        enemy2.x = -1.5
        enemy2.y = 8.2

        if self.counter % 3 == 0:
            msg.enemies = []

        elif self.counter % 3 == 1:
            msg.enemies = [enemy1]

        else:
            msg.enemies = [enemy1, enemy2]

        self.counter += 1

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published {len(msg.enemies)} enemies'
        )


def main(args=None):
    rclpy.init(args=args)

    node = FakeEnemyPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()