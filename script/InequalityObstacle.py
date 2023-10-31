#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 20 15:38:55 2022

@author: andthem
"""

from math import cos as Cos
from math import sin as Sin
from math import sqrt as Sqrt
from math import prod as Prod
from math import pi, atan2
import casadi.casadi as cs
import matplotlib.pyplot as plt
from numbers import Number
import warnings


inf = float("inf")


def dist2(A, B):
	return (A[0] - B[0])**2 + (A[1] - B[1])**2


def dist(A, B):
	return Sqrt(dist2(A, B))


class WrongInputError(Exception):
	pass


class Point:

	def __init__(self, P, Py=None):
		if Py is None:
			self.__x, self.__y = P
		else:
			self.__x, self.__y = P, Py

	def __str__(self):
		return "(%.3f, %.3f)" % (self.x, self.y)

	def __repr__(self):
		return "Point(%.3f, %.3f)" % (self.x, self.y)

	def __getitem__(self, key):
		return (self.x, self.y)[key]

	def __contains__(self, something):
		raise TypeError("Point objects do not have a __contain__ method")

	def __eq__(self, other):
		if not self.is_comparable_with(other):
			return False
		return self.x == other[0] and self.y == other[1]

	def __ne__(self, other):
		return not self == other

	def __le__(self, other):
		self.assert_is_comparable_with(other)
		return self.x <= other[0] and self.y <= other[1]

	def __lt__(self, other):
		self.assert_is_comparable_with(other)
		return self.x < other[0] and self.y < other[1]

	def __ge__(self, other):
		self.assert_is_comparable_with(other)
		return self.x >= other[0] and self.y >= other[1]

	def __gt__(self, other):
		self.assert_is_comparable_with(other)
		return self.x > other[0] and self.y > other[1]

	def __neg__(self):
		return Point((-self.x, -self.y))

	def __add__(self, v):
		self.assert_is_comparable_with(v)
		return Point((self.x + v[0], self.y + v[1]))

	def __sub__(self, v):
		self.assert_is_comparable_with(v)
		return Point((self.x - v[0], self.y - v[1]))

	def __mul__(self, c):
		if not isinstance(c, Number):
			msg = "Can only multiply Point with number "
			msg += f"(got {c} of type {type(c)} instead)"
			raise TypeError(msg)
		return Point((self.x * c, self.y * c))

	def __rmul__(self, c):
		return self * c

	def __truediv__(self, c):
		return Point((self.x / c, self.y / c))

	def is_comparable_with(self, other):
		return isinstance(other, Point) or \
			(type(other) in (tuple, list) and len(other) == 2)

	def assert_is_comparable_with(self, other):
		if not self.is_comparable_with(other):
			msg = f"Point object {self} not compatible with "
			msg += f"{type(other)} object {other}"
			raise TypeError(msg)

	def translate(self, v):
		self.__x += v[0]
		self.__y += v[1]

	def rotate(self, angle, center=(0, 0)):
		C = Point(center)
		cos = Cos(angle)
		sin = Sin(angle)
		self.translate(-C)
		self.__x, self.__y = cos * self.x - sin * self.y, sin * self.x + cos * self.y
		self.translate(C)

	def normal(self):
		return Point(self.y, -self.x) / self.norm

	def copy(self):
		return Point((self.x, self.y))

	def plot(self, *args, **kwargs):
		return plt.plot(self.__x, self.__y, *args, **kwargs)

	@property
	def x(self):
		return self.__x

	@property
	def y(self):
		return self.__y

	@property
	def norm(self):
		return dist(self, (0, 0))

	@property
	def norm2(self):
		return dist2(self, (0, 0))


# Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class Function:

	def __init__(self, func):
		if isinstance(func, Number):
			def func(p):
				return float(func)
		elif isinstance(func, Function):
			func = func._f
		self._f = func

	def __call__(self, p):
		return self._f(p)

	def __add__(self, other):
		other = Function(other)
		return Function(lambda p: self(p) + other(p))

	def __sub__(self, other):
		other = Function(other)
		return Function(lambda p: self(p) - other(p))

	def __mul__(self, other):
		other = Function(other)
		return Function(lambda p: self(p) * other(p))

	def __neg__(self):
		return Function(lambda p: -self(p))

	def __truediv__(self, other):
		other = Function(other)
		return Function(lambda p: self(p) / other(p))

	@property
	def f(self):
		return self._f


def composition(func, T):
	return Function(lambda p: func(T(p)))


# Obstacles ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class PlotDataWarning(UserWarning):
	pass


class Obstacle:
	"""
	Models (open) obstacles {x: g(x)<0 for all g in self._inequalities}
	"""
	_tol = 1e-8
	is_closed = False

	def __init__(self, center=None):
		# TODO: automatize center for polygonal obstacles
		self._center = (0, 0) if center is None else Point(center)
		self._points = []  # collects points for drawing
		self._area = None  # useful for detecting center of union of obstacles
		self._inequalities = []

	def __add__(self, func):
		self._inequalities.append(Function(func))
		return self

	def __neg__(self):
		Obs = Obstacle(center=self.center)
		if len(self._inequalities) == 1:
			g = -self._inequalities[0]
		else:
			g = -Function(self.equality_constraint())
		return Obs + g

	def __contains__(self, P):
		return all([g(P) < self._tol for g in self._inequalities])

	def intersect(self, other, updatepoints=True, warn=None):
		for func in other.inequalities:
			self += func
		if updatepoints:
			self._points = [p for p in self.points if p in self]
			self._points += [p for p in other.points if p in self]
			self._points = self.sort_points(self._points)
			self._center = self.barycenter(self._points)
			if warn is None:
				warn = True
		if warn:
			msg = "Plot data may be lost or incorrect after intersection"
			warnings.warn(msg, PlotDataWarning)

	def transform(self, T, invT=None):
		"""Performs a (linear) transformation T. Also works for nonlinear T,
		but a correct transformation of self.points requires the inverse
		transformation invT to be provided"""
		if invT is None:
			invT = self.inv(T)
		for i, g in enumerate(self.inequalities):
			self._inequalities[i] = composition(g, invT)
		self._center = T(self._center)
		self._points = [T(p) for p in self.points]
		if self._area is not None:
			self._area *= abs(self.det(T))
		if self.is_enlarged:
			self._enlarged.transform(T, invT)

	def rotate(self, angle, center=None):
		center = self.center if center is None else Point(center)
		cos = Cos(angle)
		sin = Sin(angle)
		self.translate(-center)

		def T(p):
			return (cos * p[0] - sin * p[1], sin * p[0] + cos * p[1])

		self.transform(T)
		self.translate(center)

	def translate(self, vec):
		def T(p):
			return (p[0] + vec[0], p[1] + vec[1])
		self.transform(T)

	def scale(self, r, center=None):
		try:
			rx, ry = r
		except TypeError:
			rx, ry = r, r
		center = self.center if center is None else Point(center)
		self.translate(-center)

		def T(p):
			return (rx * p[0], ry * p[1])

		self.transform(T)
		self.translate(center)

	def plot(self, *args, **kwargs):
		if not self.points:
			cls = self.__class__.__name__
			warnings.warn(f"No points to plot for {cls} object", PlotDataWarning)
			return
		x = [p[0] for p in self._points]
		y = [p[1] for p in self._points]
		if self.is_closed:
			x.append(x[0])
			y.append(y[0])
		#return plt.plot(x, y, *args, **kwargs)
		return x, y

	def equality_constraint(self):
		def f(p):
			return Prod(cs.fmin(g(p), 0)**2 for g in self.inequalities)
		return f

	@staticmethod
	def barycenter(points):
		return sum([Point(p) for p in points]) / len(points)

	@staticmethod
	def sort_points(points):
		c = Obstacle.barycenter(points)
		centered_points = [Point(p) - c for p in points]
		angles = [Obstacle.polar(p)[1] for p in centered_points]
		points = [tuple([a] + list(p)) for a, p in zip(angles, points)]
		points = [tuple(p[1:]) for p in sorted(points)]
		return points

	@staticmethod
	def polar(p):
		r = Sqrt(p[0]**2 + p[1]**2)
		theta = atan2(p[1], p[0])
		return r, theta

	@staticmethod
	def coeff(T):
		"""
		Extracts a dictionary with keys 'a','b','c','d','u','v' where
		T((x,y)) = (ax + by + u, cx + dy + v)
		"""
		u, v = T((0, 0))

		def S(p):
			return (T(p)[0] - u, T(p)[1] - v)

		a = S((1, 0))[0]
		c = S((1, 0))[1]
		b = S((0, 1))[0]
		d = S((0, 1))[1]
		return {"a": a, "b": b, "c": c, "d": d, "u": u, "v": v}

	@staticmethod
	def det(T):
		"""Determinant of linear transformation"""
		C = Obstacle.coeff(T)
		return C["a"] * C["d"] - C["b"] * C["c"]

	@staticmethod
	def inv(T):
		"""Inverse of linear transformation"""
		C = Obstacle.coeff(T)
		a, b, c, d, u, v = C["a"], C["b"], C["c"], C["d"], C["u"], C["v"]
		det = a * d - b * c

		def invT(p):
			return ((d * (p[0] - u) - b * (p[1] - v)) / det,
			        (-c * (p[0] - u) + a * (p[1] - v)) / det)

		return invT

	@property
	def center(self):
		return Point(self._center)

	@property
	def area(self):
		return self._area

	@property
	def points(self):
		return self._points[:]

	@property
	def inequalities(self):
		return self._inequalities[:]

	@property
	def enlarged(self):
		return self._enlarged

	@property
	def is_enlarged(self):
		return hasattr(self, "_enlarged")


class HalfSpace(Obstacle):
	def __init__(self, P, v, enlargement=0):
		"""Halfspace passing through P with outward normal v"""
		super().__init__()
		P = Point(P)
		v = Point(v)
		self += lambda p: (p[0] - P.x) * v.x + (p[1] - P.y) * v.y
		n = v.normal()
		self._points = [P - 100 * n, P + 100 * n]
		self._area = inf
		if enlargement:
			self._enlarged = HalfSpace(P + v * (enlargement / v.norm), v)


class Box(Obstacle):
	def __init__(self, lb=None, ub=None, enlargement=0):
		"""Box lb <= x <= ub"""
		lb = Point(-inf, -inf) if lb is None else Point(lb)
		ub = Point(inf, inf) if ub is None else Point(ub)
		if not lb < ub:
			msg = f"Box should have nonzero area (lb={lb} should be < than ub={ub})"
			raise WrongInputError(msg)
		super().__init__()
		Ps = []
		vs = []
		cx, cy = None, None
		if lb.x > -inf:
			Ps.append((lb.x, 0))
			vs.append((-1, 0))
			cx = lb.x if cx is None else (cx + lb.x) / 2
		if ub.x < inf:
			Ps.append((ub.x, 0))
			vs.append((1, 0))
			cx = ub.x if cx is None else (cx + ub.x) / 2
		if lb.y > -inf:
			Ps.append((0, lb.y))
			vs.append((0, -1))
			cy = lb.y if cy is None else (cy + lb.y) / 2
		if ub.y < inf:
			Ps.append((0, ub.y))
			vs.append((0, 1))
			cy = ub.y if cy is None else (cy + ub.y) / 2
		for P, v in zip(Ps, vs):
			self.intersect(HalfSpace(P=P, v=v), updatepoints=False)
		self._center = (cx, cy)
		if enlargement:
			eps = Point(enlargement, enlargement)
			self._enlarged = Box(lb - eps, ub + eps)


class Rectangle(Box):
	is_closed = True

	def __init__(self, A, C, enlargement=0):
		"""Horizontal rectangle with diagonal from A to C"""
		Ax, Ay = min(A[0], C[0]), min(A[1], C[1])
		Cx, Cy = max(A[0], C[0]), max(A[1], C[1])
		A = Point(Ax, Ay)
		C = Point(Cx, Cy)
		super().__init__(A, C, enlargement=enlargement)
		self._area = abs((Cx - Ax) * (Cy - Ay))
		self._points = [A, Point(Cx, Ay), C, Point(Ax, Cy)]
		if enlargement:
			eps = Point(enlargement, enlargement)
			self._enlarged = Rectangle(A - eps, C + eps)


class Square(Rectangle):
	def __init__(self, center, r, enlargement=0):
		"""Horizontal square centered at center with radius (half side) r"""
		center = Point(center)
		super().__init__(center - (r, r), center + (r, r), enlargement=enlargement)


class Segment(Rectangle):
	def __init__(self, A, B, width, enlargement=0):
		"""Thick segment between A and B"""
		A, B = Point(A), Point(B)
		M = (A + B) / 2
		w, h = dist(A, B), width
		d = Point(w, h) / 2
		super().__init__(M - d, M + d, enlargement=enlargement)
		_, angle = self.polar(B - A)
		self.rotate(angle)


class Ellipse(Obstacle):
	is_closed = True

	def __init__(self, center, r, enlargement=0):
		"""Circle centered at center with radius r"""
		super().__init__(center=center)
		try:
			rx, ry = r
		except TypeError:
			rx, ry = r, r
		cx, cy = center
		self += Function(lambda p: ((p[0] - cx) / rx) ** 2
				               + ((p[1] - cy) / ry)**2 - 1.0)
		self._area = pi * rx * ry
		k = 0
		T = []
		while k <= 2 * pi:
			T.append(k)
			k += 0.01
		self._points = [(cx + rx * Cos(t), cy + ry * Sin(t)) for t in T]
		if enlargement:
			self._enlarged = Ellipse(center, (rx + enlargement, ry + enlargement))


class InnerCurve(Obstacle):
	is_closed = True

	def __init__(self, center, r, angle=0, enlargement=0):
		"""90* counterclockwise elliptical inner curve tilted by angle"""
		if type(angle) is int:
			angle = angle * pi / 180
		super().__init__(center=center)
		E = Ellipse(center=center, r=r)
		self.intersect(E, updatepoints=False)
		self.intersect(HalfSpace(center, (0, -1)), updatepoints=False)
		self.intersect(HalfSpace(center, (-1, 0)), updatepoints=False)
		self._points = [p for p in E.points if p in self]
		self._points.append(center)
		self.rotate(angle)
		self._area = E.area / 4
		if enlargement:
			C = Point(center)
			eps = Point(enlargement, enlargement)
			try:
				re = r + 2 * enlargement
			except IndexError:
				re = r + 2 * eps
			self._enlarged = InnerCurve(C - eps, re)
			self._enlarged._center = C
			self._enlarged.rotate(angle)


class OuterCurve(Obstacle):
	is_closed = True

	def __init__(self, center, r, angle=0, enlargement=0):
		"""90* counterclockwise elliptical outer curve tilted by angle"""
		if type(angle) is int:
			angle = angle * pi / 180
		super().__init__(center=center)
		try:
			rx, ry = r
		except TypeError:
			rx, ry = r, r
		E = Ellipse(center=center, r=(rx, ry))
		C = Point(center)
		self.intersect(-E, updatepoints=False)
		self.intersect(Rectangle(C, C + (rx, ry)), updatepoints=False)
		self._points = [p for p in E.points if p in self]
		self._points.append(C + (rx, ry))
		self.rotate(angle)
		self._area = rx * ry - E.area / 4
		if enlargement:
			eps = Point(enlargement, enlargement)
			r = Point(rx, ry)
			Enlarged = Obstacle(center=C)
			Enlarged.is_closed = True
			E = Ellipse(center=C, r=r - eps)
			Enlarged.intersect(-E, updatepoints=False)
			Enlarged.intersect(Rectangle(C, C + r + eps), updatepoints=False)
			Enlarged._points = [p for p in E.points if p in Enlarged]
			Enlarged._points += [C + (0, ry + enlargement),
						         C + r + eps, C + (rx + enlargement, 0)]
			Enlarged.rotate(angle)
			Enlarged._area = (rx + enlargement) * (ry + enlargement) - E.area / 4
			self._enlarged = Enlarged


class Shape:
	"""Container for union of obstacles"""

	def __init__(self):
		self.obstacles = []

	def __add__(self, obs):
		self.obstacles.append(obs)
		return self

	def __iter__(self):
		return iter(self.obstacles)

	def __contains__(self, P):
		return any(P in obs for obs in self)


class Curve(Shape):
	def __init__(self, center, inner, outer, angle=0, enlargement=0):
		super().__init__()
		try:
			rix, riy = inner
		except TypeError:
			rix, riy = inner, inner
		try:
			rox, roy = outer
		except TypeError:
			rox, roy = outer, outer
		assert 0 < rix < rox < inf
		assert 0 < riy < roy < inf
		self += InnerCurve(center, inner, angle, enlargement)
		self += OuterCurve(center, outer, angle, enlargement)


test = True
test = False
if test:
	xmin = -1
	xmax = 4
	ymin = -1
	ymax = 4

	plt.plot([xmin, xmax], [0, 0], "k")
	plt.plot([0, 0], [ymin, ymax], "k")
	plt.xlim(xmin, xmax)
	plt.ylim(ymin, ymax)
	plt.grid()
	plt.axis("equal")

	s = Segment((0, 1), (2, 0), width=0.5, enlargement=0.25)
	se = s.enlarged

	h = s.equality_constraint()
	he = se.equality_constraint()
	s.plot("b")
	se.plot("r--")

	for t in [x / 10 for x in range(40)]:
		p = (t, t)
		if p in s:
			plt.plot(t, t, "b*")
			print(t, "%.3f*" % h(p), "%.3f" % he(p))
			if p not in se:
				print("^^^^^ ERROR")
		elif p in se:
			plt.plot(t, t, "ro")
			print(t, "%.3f " % h(p), "%.3f*" % he(p))
