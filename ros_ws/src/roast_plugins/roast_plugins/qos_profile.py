"""QOS Profile for the Roast robot"""
# Copyright 2022 Franklin Selva. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

ROAST_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
