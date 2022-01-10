#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
import struct
import time

from sensor_msgs.msg import PointCloud2

class lidar_collect(Node):
	
	def __init__(self):
		super().__init__("lidar_collect")
		self.subscriber_lidar = self.create_subscription(PointCloud2, "velodyne_points", self.record, 50)
		self.scan = 0

	def record(self, msg):

		index = 0
		temp = []
		temp.append([0,0,0,time.time()])
		while index < len(msg.data):
			x = struct.unpack('<f',struct.pack('4B', *msg.data[index + 0: index + 4]))
			y = struct.unpack('<f',struct.pack('4B', *msg.data[index + 4: index + 8]))
			z = struct.unpack('<f',struct.pack('4B', *msg.data[index + 8: index + 12]))
			intensity = struct.unpack('<f',struct.pack('4B', *msg.data[index + 12: index + 16]))
			coordinate = [x[0],y[0],z[0],intensity[0]]
			index = index + 18
			if coordinate[0] != coordinate[0]:
				continue
			temp.append(coordinate)

		temp = np.array(temp)
		np.save('/media/amm/Backup/acads/ARTPARK/test/transvahan/Data/2021-12-14/run2_lidar/%05d.npy'%(self.scan), temp)
		self.scan = self.scan + 1

def main(args=None):

	rclpy.init(args=args)
	node = lidar_collect()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		rclpy.shutdown()

if __name__ == '__main__':
	main()