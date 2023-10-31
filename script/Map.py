#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 18 17:13:59 2022

@author: andthem
"""

import InequalityObstacle


class Grid:
	def __init__(self):
		self.__grid = {}
		self.__xmin, self.__xmax, self.__ymin, self.__ymax = (None,) * 4

	def __getitem__(self, pos):
		x, y = pos
		return self.__grid[x][y]

	def __setitem__(self, pos, val):
		x, y = pos
		try:
			Dx = self.__grid[x]
		except KeyError:
			Dx = {}
		Dx[y] = val
		self.__xmin, self.__xmax = (x, x) if self.__xmin is None else (
			min(self.__xmin, x), max(self.__xmax, x))
		self.__ymin, self.__ymax = (y, y) if self.__ymin is None else (
			min(self.__ymin, y), max(self.__ymax, y))

	@property
	def size(self):
		return ((self.__xmin, self.__xmax), (self.__ymin, self.__ymax))


class Map:

	def __init__(self, enlargement=0):
		self.obstacles = []
		self.agents = []
		self.enlargement = enlargement

	def ID(self):
		raise NotImplementedError

	def add_obstacle(self, cls, *args, **kwargs):
		obs = cls(*args, **kwargs, enlargement=self.enlargement)
		Obs = obs if isinstance(obs, InequalityObstacle.Shape) else [obs]
		for obs in Obs:
			self.obstacles.append(obs)

	def add_agent(self, ag):
		self.agents.append(ag)
