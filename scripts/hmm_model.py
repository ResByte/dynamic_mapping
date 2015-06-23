#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  hmm_model.py
#  
#  Copyright 2014 abhinav <abhinav@abhinav-VirtualBox>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import rospy
from dynamic_mapping.srv import hmm_srv
from ghmm import *


def callback(data):
	print data.num1


def hmmProcess(req):
	A = [[0.9, 0.1], [0.3, 0.7]]
	pi = [0.5] * 2
	static_obj = [0.5,0.5]
	dynamic_obj = [0.1,0.9]
	B = [static_obj,dynamic_obj]
	sigma = IntegerRange(1,3)
	m=HMMFromMatrices(sigma,DiscreteDistribution(sigma),A,B,pi)
	print m
	if req.b > 0.5:
		var1 = 2
	else:
		var1 = 1
	if req.a > 0.5:
		var2 = 2
	else:
		var2 = 1
	seq = EmissionSequence(sigma,[var1,var2])
	print m.viterbi(seq)
	return 19.0

def main():
	rospy.init_node('hmm_model',anonymous = True)
	s = rospy.Service('hmm_data',hmm_srv,hmmProcess)
	#rospy.Subscriber("Hmm_data",Num,callback)
	rospy.spin()
	
	return 0

if __name__ == '__main__':
	main()
