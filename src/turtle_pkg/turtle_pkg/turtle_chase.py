import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from turtlesim.srv import Spawn , Kill
from functools import partial
import random
import numpy as np


class TurtleChase(Node):

    def __init__(self):
        super().__init__('turtle_chase')
        self.x=-100000
        self.y=self.x
        self.enemy_positions=dict()
        self.score=0
        
        for i in range(0,3) :
            self.spawn_enemy("enemy" + str(i+1))
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.player_callback,
            10)
        


        self.subscription_enemy1 = self.create_subscription(
            Pose,
            '/enemy1/pose',
            partial(self.enemy_callback,name = 'enemy1'),
            10)
        
        self.subscription_enemy2 = self.create_subscription(
            Pose,
            '/enemy2/pose',
            partial(self.enemy_callback,name = 'enemy2'),
            10)
        
        self.subscription_enemy3 = self.create_subscription(
            Pose,
            '/enemy3/pose',
            partial(self.enemy_callback,name = 'enemy3'),
            10)
        self.publisher_ = self.create_publisher(Int32, '/score', 10)
        self.score_callback()
        self.create_timer(0.1,self.check_collisions)


    def spawn_enemy(self,name):
        client=self.create_client(Spawn,"/spawn")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting...")

        self.request=Spawn.Request()
        x=random.uniform(1.0,10.0)
        y=random.uniform(1.0,10.0)
        self.request.x=x
        self.request.y=y
        self.request.theta=0.0
        self.request.name=name
        future=client.call_async(self.request)

        future.add_done_callback(partial(self.Spawn_callback,x=x,y=y)) 
    def Spawn_callback(self,future,x,y):
        try:
            response =future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))
    

    def call_service_Kill(self,name : str):
        client=self.create_client(Kill,"/kill")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting...")

        request=Kill.Request()
        request.name=name
        future=client.call_async(request)
        future.add_done_callback(partial(self.Kill_callback,name=name)) 

        
    def Kill_callback(self,future,name : str):
        try:
            response=future.result()
            self.enemy_positions.pop(name)
            self.spawn_enemy(name)
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))


    
    def score_callback(self):
        msg=Int32() 
        msg.data=self.score
        self.publisher_.publish(msg)


            
    def player_callback(self, msg:Pose):
                self.msg=msg


    def enemy_callback(self, msg:Pose,name : str) :
        self.enemy_positions[name]=msg
    def check_collisions(self):
        if not hasattr(self, "msg"):
            return
        for name, pose in list(self.enemy_positions.items()):
            dist=find_distance(pose,self.msg)
            if(dist < 0.5) :
                self.call_service_Kill(name)
                self.score+=1
                self.score_callback()




def find_distance(pose1: Pose,pose2: Pose) -> float :
    return np.sqrt((pose1.x-pose2.x)**2 + (pose1.y-pose2.y)**2)
def main(args=None):
    rclpy.init(args=args)
    node = TurtleChase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__" :
    main()