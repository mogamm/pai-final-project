{\rtf1\ansi\ansicpg932\cocoartf2870
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 #!/usr/bin/env python3\
import rclpy\
from rclpy.node import Node\
from sensor_msgs.msg import JointState\
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint\
import torch\
import argparse\
import numpy as np\
from lerobot.common.policies.act.modeling_act import ACTPolicy\
\
# \uc0\u12452 \u12531 \u12487 \u12483 \u12463 \u12473 \u38918 : [0:joint1(\u26059 \u22238 ), 1:joint2(\u32937 ), 2:joint4(\u25163 \u39318 ), 3:joint3(\u32920 )]\
ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']\
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'\
\
class DynamicAutonomousNode(Node):\
    def __init__(self, model_path):\
        super().__init__('crane_plus_dynamic_autonomous')\
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)\
        self.gripper_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)\
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)\
        \
        print("==================================================")\
        print(" [\uc0\u36215 \u21205 ] ACT\u12514 \u12487 \u12523 \u12434 \u35501 \u12415 \u36796 \u12435 \u12391 \u12356 \u12414 \u12377 ...")\
        self.policy = ACTPolicy.from_pretrained(model_path)\
        self.policy.eval()\
        self.device = torch.device('cpu')\
        self.policy.to(self.device)\
        print(" [\uc0\u23436 \u20102 ] \u33258 \u24459 \u12479 \u12473 \u12463 \u65288 \u27491 \u38754 \u12408 \u12398 \u28145 \u12356 \u12362 \u36766 \u20736 \u65286 \u24195 \u35282 \u12473 \u12461 \u12515 \u12531 \u65289 \u12434 \u38283 \u22987 \u12375 \u12414 \u12377 \u12290 ")\
        print("==================================================")\
\
        self.latest_joint_state = None\
        self.state = 'INIT_WAIT'\
        self.step_counter = 0\
        self.cycle_count = 1\
        \
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz\
\
    def joint_state_callback(self, msg):\
        name_to_pos = dict(zip(msg.name, msg.position))\
        arm_pos = [name_to_pos.get(j, 0.0) for j in ARM_JOINT_NAMES]\
        gripper_pos = name_to_pos.get(GRIPPER_JOINT_NAME, 0.0)\
        self.latest_joint_state = np.array(arm_pos + [gripper_pos], dtype=np.float32)\
\
    def send_command(self, arm_positions, gripper_position, duration_sec=0.5):\
        jt = JointTrajectory()\
        jt.joint_names = ARM_JOINT_NAMES\
        pt = JointTrajectoryPoint()\
        pt.positions = [float(x) for x in arm_positions]\
        pt.time_from_start.sec = int(duration_sec)\
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)\
        jt.points.append(pt)\
        self.arm_pub.publish(jt)\
\
        jt_g = JointTrajectory()\
        jt_g.joint_names = [GRIPPER_JOINT_NAME]\
        pt_g = JointTrajectoryPoint()\
        pt_g.positions = [float(gripper_position)]\
        pt_g.time_from_start.sec = int(duration_sec)\
        pt_g.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)\
        jt_g.points.append(pt_g)\
        self.gripper_pub.publish(jt_g)\
\
    def control_loop(self):\
        if self.latest_joint_state is None:\
            return\
\
        self.step_counter += 1\
        current_j1 = self.latest_joint_state[0]\
        current_j3 = self.latest_joint_state[3]\
\
        # 1. \uc0\u30452 \u31435 \u24453 \u27231  (\u26368 \u21021 \u12398 1\u22238 \u12384 \u12369 \u25351 \u20196 \u12434 \u36865 \u12426 \u12289 \u30906 \u23455 \u12395 \u30452 \u31435 \u12373 \u12379 \u12427 )\
        if self.state == 'INIT_WAIT':\
            if self.step_counter == 1:\
                print(f"\\n--- [\uc0\u12469 \u12452 \u12463 \u12523  \{self.cycle_count\}] 1. \u24453 \u27231 \u29366 \u24907 \u65288 \u30452 \u31435 \u23039 \u21218 \u12408 \u12522 \u12475 \u12483 \u12488 \u65289  ---")\
                self.send_command([0.0, 0.0, 0.0, 0.0], -0.5, duration_sec=1.0)\
            if self.step_counter >= 40: # 2\uc0\u31186 \u24460 \u12395 \u38283 \u22987 \
                print(f"--- [\uc0\u12469 \u12452 \u12463 \u12523  \{self.cycle_count\}] 2. ACT\u12514 \u12487 \u12523 \u25512 \u35542 \u12395 \u12424 \u12427 \u28145 \u12356 \u12362 \u36766 \u20736 \u12434 \u38283 \u22987  ---")\
                self.policy.reset()\
                self.state = 'AI_BOWING'\
                self.step_counter = 0\
\
        # 2. AI\uc0\u25512 \u35542 \u12395 \u12424 \u12427 \u27491 \u38754 \u12408 \u12398 \u28145 \u12356 \u12362 \u36766 \u20736 \
        elif self.state == 'AI_BOWING':\
            obs = \{\
                'observation.state': torch.tensor(self.latest_joint_state, dtype=torch.float32).unsqueeze(0),\
                'observation.environment_state': torch.tensor(self.latest_joint_state, dtype=torch.float32).unsqueeze(0),\
            \}\
            with torch.no_grad():\
                action = self.policy.select_action(obs).cpu().numpy().flatten()\
\
            # \uc0\u23398 \u32722 \u12375 \u12383 joint3\u12398 \u25512 \u35542 \u20516 (\u32004 +0.49)\u12434 \u12505 \u12540 \u12473 \u12395 \u12289 \u21516 \u12376 \u26041 \u21521 (\u12503 \u12521 \u12473 \u20596 )\u12408 \u20840 \u38306 \u31680 \u12434 \u36899 \u21205 \u12373 \u12379 \u12390 \u33258 \u28982 \u12394 \u24359 \u12434 \u25551 \u12367 \
            ai_val = max(float(action[3]), 0.35)\
            target_j2 = ai_val * 0.4    # \uc0\u32937 \u12434 \u23569 \u12375 \u21069 \u12395 \u20670 \u12369 \u12427  (+0.2 rad)\
            target_j3 = ai_val * 1.8    # \uc0\u32920 \u12434 \u12513 \u12452 \u12531 \u12391 \u28145 \u12367 \u21069 \u12395 \u26354 \u12370 \u12427  (+0.9 rad)\
            target_j4 = ai_val * 0.8    # \uc0\u25163 \u39318 \u12418 \u21516 \u26041 \u21521 \u12395 \u26354 \u12370 \u12390 \u32186 \u40599 \u12394 \u31036 \u12395 \u12377 \u12427  (+0.4 rad)\
            \
            if self.step_counter == 1 or self.step_counter % 10 == 0:\
                self.send_command([0.0, target_j2, target_j4, target_j3], 0.5, duration_sec=0.45)\
                print(f"  [AI\uc0\u25512 \u35542 \u20013 ] AI\u20986 \u21147 (j3): \{action[3]:.3f\} -> \u12362 \u36766 \u20736 \u30446 \u27161 (j3: \{target_j3:.2f\}, j4: \{target_j4:.2f\})")\
\
            if self.step_counter >= 50: # 2.5\uc0\u31186 \u12391 \u12362 \u36766 \u20736 \u23436 \u20102 \
                print(f"--- [\uc0\u12469 \u12452 \u12463 \u12523  \{self.cycle_count\}] 3. \u28145 \u12356 \u12362 \u36766 \u20736 \u23436 \u20102 \u65281 \u23039 \u21218 \u12434 \u12461 \u12540 \u12503 \u12375 \u12414 \u12377  ---")\
                self.state = 'HOLD_BOW'\
                self.step_counter = 0\
\
        # 3. \uc0\u28145 \u12356 \u12362 \u36766 \u20736 \u23039 \u21218 \u12434 \u12461 \u12540 \u12503  (\u32004 1.5\u31186 )\
        elif self.state == 'HOLD_BOW':\
            if self.step_counter == 1:\
                self.send_command([0.0, 0.20, 0.45, 0.95], 0.5, duration_sec=0.5)\
            if self.step_counter >= 30:\
                print(f"--- [\uc0\u12469 \u12452 \u12463 \u12523  \{self.cycle_count\}] 4. \u30452 \u31435 \u23039 \u21218 \u12408 \u24489 \u24112 \u12375 \u12414 \u12377  ---")\
                self.state = 'RETURN_UP'\
                self.step_counter = 0\
\
        # 4. \uc0\u30452 \u31435 \u12408 \u25147 \u12427  (\u32004 2\u31186 )\
        elif self.state == 'RETURN_UP':\
            if self.step_counter == 1:\
                self.send_command([0.0, 0.0, 0.0, 0.0], -0.5, duration_sec=1.0)\
            if self.step_counter >= 40:\
                print(f"--- [\uc0\u12469 \u12452 \u12463 \u12523  \{self.cycle_count\}] 5. \u24195 \u35282 \u12473 \u12461 \u12515 \u12531 \u38283 \u22987 \u65288 \u24038 \u21491 \u12408 \u32004 80\u24230 \u12378 \u12388 \u22823 \u12365 \u12367 \u26059 \u22238 \u65289  ---")\
                self.state = 'LOOK_AROUND'\
                self.step_counter = 0\
\
        # 5. \uc0\u24038 \u21491 \u12434 \u22823 \u12365 \u12367 \u35211 \u28193 \u12377 \u21205 \u12365  (\'b11.4 rad \u8786  \u24038 \u21491 \u32004 80\u24230 \u12378 \u12388 )\
        elif self.state == 'LOOK_AROUND':\
            if self.step_counter == 1:\
                print("  [\uc0\u21608 \u22258 \u30906 \u35469 ] \u24038 \u26041 \u21521 \u12408 \u22823 \u12365 \u12367 \u26059 \u22238 \u20013  (+1.4 rad / \u32004 80\u24230 )...")\
                self.send_command([1.4, 0.0, 0.0, 0.0], 0.3, duration_sec=1.2)\
            elif self.step_counter == 40:\
                print(f"  [\uc0\u21608 \u22258 \u30906 \u35469 ] (\u29694 \u22312 \u26059 \u22238 \u35282 : \{current_j1:.2f\} rad) -> \u21491 \u26041 \u21521 \u12408 \u22823 \u12365 \u12367 \u26059 \u22238 \u20013  (-1.4 rad / \u32004 80\u24230 )...")\
                self.send_command([-1.4, 0.0, 0.0, 0.0], 0.3, duration_sec=1.5)\
            \
            if self.step_counter >= 85:\
                print(f"  [\uc0\u21608 \u22258 \u30906 \u35469 ] \u12473 \u12461 \u12515 \u12531 \u23436 \u20102  (\u29694 \u22312 \u26059 \u22238 \u35282 : \{current_j1:.2f\} rad)")\
                self.cycle_count += 1\
                self.state = 'INIT_WAIT'\
                self.step_counter = 0\
\
def main():\
    parser = argparse.ArgumentParser(description='Dynamic Autonomous Task with ACTPolicy')\
    parser.add_argument('--model-path', type=str, required=True, help='Path to trained ACTPolicy model directory')\
    args = parser.parse_args()\
    rclpy.init()\
    node = DynamicAutonomousNode(args.model_path)\
    try:\
        rclpy.spin(node)\
    except KeyboardInterrupt:\
        print("\\n[\uc0\u32066 \u20102 ] \u12503 \u12525 \u12464 \u12521 \u12512 \u12434 \u20572 \u27490 \u12375 \u12414 \u12375 \u12383 \u12290 ")\
    finally:\
        node.destroy_node()\
        rclpy.shutdown()\
\
if __name__ == '__main__':\
    main()}