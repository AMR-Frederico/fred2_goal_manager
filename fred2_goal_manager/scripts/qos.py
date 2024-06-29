#!/usr/bin/env python3

from rclpy.node import Node

from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


def general_config():

    # quality protocol -> the node can't lose any message 
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE, 
        durability= QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST, 
        depth=10, 
        liveliness=QoSLivelinessPolicy.AUTOMATIC
    )

    return qos_profile