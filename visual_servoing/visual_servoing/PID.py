from rclpy.node import Node

class PID:
    def __init__(self, node: Node, p, i, d):
        self.node = node
        self.p = p
        self.i = i
        self.d = d
        self.sum = 0
        self.last = None
        self.last_t = None
        self.control = None

    def __call__(self, err):
        t = self.node.get_clock().now().nanoseconds

        if self.last_t is None:
            self.last_t = t
            self.last = err
            return 0

        dt = 1e-9 * (t - self.last_t)
        derr = err - self.last

        self.sum += err * dt
        self.last = err
        self.last_t = t

        self.control = self.p * err + self.i * self.sum + self.d * (derr / dt)

        return self.control