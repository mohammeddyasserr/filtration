#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import csv
import os

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.sub = self.create_subscription(Float32, '/current_speed', self.speed_callback, 10)
        self.pub = self.create_publisher(Float32, '/cmd_vel', 10)

        self.Kp = 0.8
        self.Ki = 0.25
        self.Kd = 0.1

        self.target_kph = 60.0    
        self.max_accel = 5.0 
        self.integrator_limit = 20.0

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

        self.times = []
        self.actual_speeds = []
        self.target_speeds = []
        self.start_time = None

    def speed_callback(self, msg: Float32):
        now = self.get_clock().now()
        
        if self.start_time is None:
            self.start_time = now

        if self.last_time is None:
            dt = 0.05
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt <= 0.0 or dt > 1.0:
                dt = 0.05
        self.last_time = now

        elapsed = (now - self.start_time).nanoseconds * 1e-9

        measured_kph = float(msg.data)
        measured = measured_kph / 3.6 #from topic
        target = self.target_kph / 3.6 #60kph

        error = target - measured
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error


        self.integral += error * dt
        if self.integral > self.integrator_limit:
            self.integral = self.integrator_limit
        elif self.integral < -self.integrator_limit:
            self.integral = -self.integrator_limit

        a_unsat = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        a = max(min(a_unsat, self.max_accel), -self.max_accel) 

        if (abs(a) >= self.max_accel) and (abs(a_unsat) > abs(a)):
            self.integral -= error * dt 

        control = a / self.max_accel
        if control > 1.0:
            control = 1.0
        elif control < -1.0:
            control = -1.0

        out = Float32()
        out.data = float(control)
        self.pub.publish(out)

        self.times.append(elapsed)
        self.actual_speeds.append(msg.data)
        self.target_speeds.append(self.target_kph)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        plt.figure(facecolor='#222946')
        plt.plot(node.times, node.actual_speeds, label='Actual Speed', color='#06F7FF',linewidth=1.5)   
        plt.plot(node.times, node.target_speeds, label='Target Speed', color='#02FE03',linestyle='--')   
        ax = plt.gca()
        ax.set_facecolor('#222946')
        plt.xlabel('Time (s)',color='#A8ADB1')
        plt.xlim(left=0)
        plt.ylim(bottom=0)
        plt.xticks(range(5, int(max(node.times))+5, 5),color='#A8ADB1')
        plt.yticks(range(10, int(max(node.target_speeds))+10, 10),color='#A8ADB1')
        plt.ylabel('Speed (km/h)' , color='#A8ADB1')
        plt.title('Target speed Vs Actual Speed Vs Time',color='#A8ADB1')
        plt.legend(loc='lower right')
        
        plt.grid(True,alpha=0.1,color='#A8ADB1')

        if os.path.exists('speed_plot.png'):
            os.remove('speed_plot.png')
        plt.savefig('speed_plot.png')
        node.get_logger().info('Saved plot to speed_plot.png')


        with open('speed_data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time (s)', 'Actual Speed (km/h)', 'Target Speed (km/h)'])
            for t, actual, target in zip(node.times, node.actual_speeds, node.target_speeds):
                writer.writerow([f'{t:.2f}', f'{actual:.2f}', f'{target:.2f}'])
        node.get_logger().info('Saved data to speed_data.csv')

        rclpy.shutdown()

if __name__ == '__main__':
    main()
