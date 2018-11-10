#!/usr/bin/env python
# -*- coding: utf-8 -*-

import openravepy
import numpy as np
from Queue import PriorityQueue
if not __openravepy_build_doc__:
    from openravepy import *


class Node:
    def __init__(self,config_in, parent_in):
        self.config = config_in
        self.parent = parent_in


def extend(Tree,q):
	dmin = 1000
	for node in Tree:
		dtemp = np.linalg.norm(node.config-q)
		if dtemp < dmin:
			dmin = dtemp
			q_near = node
	direction = q-q_near.config
	distance = np.linalg.norm(direction)
	if distance < step:
		q_new = q
	else:
		direction = direction/distance
		q_new = q_near.config + step*direction
		indx = (q_new < -np.pi) + (q_new > np.pi)
		if np.sum(indx)>0:
			direction = direction - 2*indx*direction
			q_new = q_near.config + step*direction/np.linalg.norm(direction)
	robot.SetActiveDOFValues(q_new)		
	if not env.CheckCollision(robot):
		Tree.append(Node(q_new,q_near))
		if (q_new == q).all():
			return Tree,0
		else:
			return Tree,1
	return Tree,-1


def connect(Tree,q):
	while 1:
		Tree,state = extend(Tree,q)
		if state != 1:
			break
	return Tree,state


def path(Tree_a,Tree_b,goalconfig):
	path = []
	node = Tree_b[-1]
	while node.parent is not None:
		path.append(node.config)
		node = node.parent
	path.append(node.config)
	path.reverse()
	path.pop()
	node = Tree_a[-1]
	while node.parent is not None:
		path.append(node.config)
		node = node.parent
	path.append(node.config)
	if (path[0]==goalconfig).all():
		path.reverse()
	return path



def rrt(env_in,robot_in,goalconfig,initialconfig,step_in,goal_bias):

	global robot,env,step
	robot = robot_in
	env = env_in
	step = step_in

	initialconfig = np.array(initialconfig)
	goalconfig = np.array(goalconfig)
	D = len(initialconfig)
	start = Node(initialconfig,None)
	end = Node(goalconfig,None)
	Tree_a = []
	Tree_b = []
	K = 10000
	Tree_a.append(start)
	Tree_b.append(end)
	for i in range(K):
		if np.random.rand(1)>goal_bias:
			q_rand = np.random.randn(D)+initialconfig
		else:
			q_rand = Tree_b[-1].config
		Tree_a,state_a = extend(Tree_a,q_rand)
		if state_a != -1:
			Tree_b,state_b = connect(Tree_b,Tree_a[-1].config)
			if state_b == 0:
				return path(Tree_a,Tree_b,goalconfig)
		tmp = Tree_a
		Tree_a = Tree_b
		Tree_b = tmp
	return None


def smooth(env_in,robot_in,path,iterations,step_in):
	global robot,env,step
	robot = robot_in
	env = env_in
	step = step_in

	for _ in range(iterations):
		l = len(path)
		a = np.random.randint(l-2)
		b = np.random.randint(l-2)
		if b == a:
			b = np.random.randint(l-1)
		p = np.random.rand(1)
		q = np.random.rand(1)
		if a > b:
			tmp = a
			a = b
			b = tmp
		conf1 = path[a]+p*(path[a+1]-path[a])
		conf2 = path[b]+q*(path[b+1]-path[b])
		if check_edge(conf1,conf2):
			path_new = []
			for i in range(a+1):
				path_new.append(path[i])
			path_new.append(conf1)
			path_new.append(conf2)
			for i in range(b+1,l):
				path_new.append(path[i])
			path = path_new
	return path


def check_edge(c1,c2):
	direction = c2-c1
	distance = np.linalg.norm(direction)
	if distance<step:
		robot.SetActiveDOFValues(c1)		
		if not env.CheckCollision(robot):
			robot.SetActiveDOFValues(c2)		
			if not env.CheckCollision(robot):
				return 1
			else:
				return 0
		else:
			return 0
	else:
		flag = 1
		for i in range(int(distance/step)+1):
			robot.SetActiveDOFValues(c1+i*step*direction/distance)		
			if env.CheckCollision(robot):
				flag = 0
		robot.SetActiveDOFValues(c2)		
		if env.CheckCollision(robot):
			flag = 0
		return flag



if __name__ == '__main__':
	print 'Please call rrt_template.py!'