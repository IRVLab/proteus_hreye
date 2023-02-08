#! /data/proteus_ws/proteus/bin/python

import rospy

from std_msgs.msg import String, Header, ColorRGBA
from proteus_hreye.msg import HREyeState
from yolact_ros_msgs.msg import Detections, Detection


class PassiveTrashDisplay(object):
    modes = ['target', 'max_conf', "sextant"]
    color_map = {'Cup':(0,0,150), 'Mug':(150,150,0), 'Bottle':(150,0,0), 'Starfish':(150,0,150), 'Can':(0,150,0), 'Bag':(150,150,150)}
    index_map = {'Cup': [0,1,2,3,37,38,39], 'Mug': [4,5,6,7,34,35,36], 'Bottle': [8,9,10,11,32,33], 'Starfish':[12,13,14,15,29,30,31], 'Can':[16,17,18,19,27,28], 'Bag':[20,21,22,23,24,25,26]}

    def __init__(self, mode="sextant"):
        rospy.init_node("trash_hreye_display")

        self.mode = mode
        self.target = None
        self.last_detections = None

        self.detections_sub = rospy.Subscriber ("/yolact_ros/detections", Detections, self.detection_cb)
        self.control_sub = rospy.Subscriber("/loco/proteus/hreye/passive/display_mode", String, self.mode_cb)
        self.hreye_pub = rospy.Publisher("/loco/proteus/hreye/state", HREyeState)


    def detection_cb(self, msg):
        self.last_detections = msg.detections

    def mode_cb(self, msg):
        mode_type = msg.data.split(':')[0]
        if mode_type in PassiveTrashDisplay.modes:
            self.mode = mode_type
            if mode_type == "target":
                self.target =  msg.data.split(':')[1]
                rospy.loginfo(f"Switched to display mode {self.mode} with target {self.target}")

                if self.target not in PassiveTrashDisplay.color_map.keys():
                    rospy.logwarn(f"Classname {self.target} not recognized, make sure your class name is correct and capitalized.")
            else:
                rospy.loginfo(f"Switched to display mode {self.mode}")
        else:
            rospy.logwarn(f"Display mode {msg.data} not recognized")

    def update_display(self):
        state_msg = HREyeState()
        state_msg.header = Header()
        state_msg.hreye_index = 1

        if self.last_detections is not None:
            if self.mode == "target":
                state_msg = self.create_target_message(state_msg)
            elif self.mode == "max_conf":
                state_msg = self.create_maxconf_message(state_msg)
            elif self.mode == "sextant":
                state_msg = self.create_sextant_message(state_msg)

        self.hreye_pub.publish(state_msg)

    def create_maxconf_message(self, msg):
        max_conf = 0.0
        max_det = None
        for det in self.last_detections:
            if det.score > max_conf:
                max_det = det
                max_conf = det.score
        if max_det is not None:
            conf = max_det.score
            color = PassiveTrashDisplay.color_map[max_det.class_name]

            for k in range(40):
                msg.state[k].r = color[0]
                msg.state[k].g = color[1]
                msg.state[k].b = color[2]
                msg.state[k].a = conf

            return msg

        else:
            return msg

    def create_sextant_message(self, msg):
        class_state = {'Cup':0.0, 'Mug':0.0, 'Bottle':0.0, 'Starfish':0.0, 'Can':0.0, 'Bag':0.0}
        for det in self.last_detections:
            current_max = class_state[det.class_name]
            if det.score > current_max and det.score > 0.4:
                class_state[det.class_name] = det.score

        for name, conf in class_state.items():
            if conf > 0.0:
                indexes = PassiveTrashDisplay.index_map[name]
                color = PassiveTrashDisplay.color_map[name]
                for k in indexes:
                    msg.state[k].r = color[0]
                    msg.state[k].g = color[1]
                    msg.state[k].b = color[2]
                    msg.state[k].a = conf

                msg.state[0].a = max(class_state.values())
        return msg
        
    
    def create_target_message(self, msg):
        max_conf = 0.0
        max_det = None
        for det in self.last_detections:
            if det.class_name == self.target and det.score > max_conf:
                max_det = det
                max_conf = det.score
        if max_det is not None:
            conf = max_det.score
            color = PassiveTrashDisplay.color_map[max_det.class_name]

            for k in range(40):
                msg.state[k].r = color[0]
                msg.state[k].g = color[1]
                msg.state[k].b = color[2]
                msg.state[k].a = conf

            return msg

        else:
            return msg

if __name__ == "__main__":
    ptd = PassiveTrashDisplay()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        ptd.update_display()
        r.sleep()
