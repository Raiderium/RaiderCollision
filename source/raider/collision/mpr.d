module raider.collision.mpr;

import raider.collision.shape;
import raider.math;
import raider.tools.reference;
import std.math;

//This file is based on mpr.c from libccd by Daniel Fiser

immutable double mprTolerance = 1E-6F;
immutable uint mprLoopMax = 100;

/**
 * Minkowski portal refinement penetration algorithm
 * 
 * Finds the points of deepest penetration between 
 * shapes A and B, and puts the results in pa and 
 * pb. pa is on A's surface and pb is on B's surface.
 * (..almost. MPR does not find the minimum separation.)
 * 
 * Returns true if an intersection was found, false 
 * if not (or if the search failed / took too long).
 */
bool mpr(U)(Shape!U A, Shape!U B, ref vec3 pa, ref vec3 pb)
{
	Simplex!U portal;
	int r = portal.discoverPortal(A, B);
	switch(r)
	{
		case 1: //FindPenetrationTouch
		case 2: //FindPenetrationSegment
			pa = portal.supports[1].Ap;
			pb = portal.supports[1].Bp;
			break;
		case 0:
			if(portal.refinePortal(A, B) < 0) return false;
			portal.findPenetration(A, B, pa, pb);
			break;
		default:
			return false;
	}
	return true;
}

unittest
{
	import raider.collision.space;

	auto space = New!(Space!int)();

	{
		auto a = New!(Sphere!int)(space, 1.0);
		auto b = New!(Sphere!int)(space, 1.0);
		a.pos = vec3(1.9,0,0);
		b.pos = vec3(0,0,0);

		vec3 Ar, Br;
		assert(mpr(a, b, Ar, Br));
		assert(mprIntersect(a, b));
		b.pos = vec3(-0.2, 0, 0);
		assert(!mprIntersect(a, b));
	}
}

bool mprIntersect(U)(Shape!U A, Shape!U B)
{
	Simplex!U portal;
	int r = portal.discoverPortal(A, B);

	if(r < 0) return false;
	if(r > 0) return true;
	r = portal.refinePortal(A, B);
	return r == 0 ? true : false;
}

struct Support(U)
{
	vec3 p; ///Support point in minkowski sum
	vec3 Ap; ///Support point in A
	vec3 Bp; ///Support point in B

	/**
	 * Computes support point of A and B in the specified direction.
	 */
	this(Shape!U A, Shape!U B, vec3 dir)
	{
		Ap = A.support(dir);
		Bp = B.support(-dir);
		p = Ap - Bp;
	}

	/**
	 * Computes an origin for the minkowski difference of A and B (any interior point)
	 */
	void findOrigin(Shape!U A, Shape!U B)
	{
		Ap = A.pos;
		Bp = B.pos;
		p = A.pos - B.pos;
	}
}

/**
 * A set of supports
 */
struct Simplex(U)
{
	alias Support!U Sup;

	Sup[4] supports; 
	private int last = -1;
	@property int size() { return last + 1; }
	@property void size(int) { last = size-1; }
	@property auto lastSupport() { return supports[last]; }
	void add(ref in Sup support) { supports[++last] = support; }
	void swap(int s0, int s1)
	{
		auto temp = supports[s0];
		supports[s0] = supports[s1];
		supports[s1] = temp;
	}

	/** 
	 * Find simplex intersecting with line from minkowski difference origin to 0,0,0.
	 * 
	 * Returns 0 on success.
	 * Returns -1 if origin is outside portal
	 * Returns 1 if origin is on support 1
	 * Returns 2 if origin is on segment from support 0 to support 1
	 */
	int discoverPortal(Shape!U A, Shape!U B)
	{
		//Support 0 (at origin)
		supports[0].findOrigin(A, B); size = 1;
		if(supports[0].p.isZero) supports[0].p[0] += 0.00001;

		//Support 1 (points at origin)
		vec3 dir = -supports[0].p;
		dir.normalize;
		supports[1] = Sup(A, B, dir); size = 2;
		if(dot(supports[1].p, dir) <= 0.0) return -1;

		//Support 2
		dir = cross(supports[0].p, supports[1].p);
		if(dir.isZero) return supports[1].p.isZero ? 1 : 2;
		dir.normalize;
		supports[2] = Sup(A, B, dir); size = 3;
		if(dot(supports[2].p, dir) <= 0.0) return -1;

		//Support 3
		vec3 va = supports[1].p - supports[0].p;
		vec3 vb = supports[2].p - supports[0].p;
		dir = cross(va, vb);
		dir.normalize;

		if(dot(supports[0].p, dir) > 0.0) { swap(1, 2); dir *= -1; }

		bool outside = false;

		while(true)
		{
			if(outside)
			{
				va = supports[1].p - supports[0].p;
				vb = supports[2].p - supports[0].p;
				dir = cross(va, vb);
				dir.normalize;
			}
			outside = false;

			supports[3] = Sup(A, B, dir);
			if(dot(supports[3].p, dir) <= 0.0) return -1;

			//Origin outside 1,0,3 => 2=3
			va = cross(supports[1].p, supports[3].p);
			if(dot(va, supports[0].p) < -double.epsilon)
			{
				supports[2] = supports[3];
				outside = true; continue;
			}

			//Origin outside 3,0,2 => 1=3
			va = cross(supports[3].p, supports[2].p);
			if(dot(va, supports[0].p) < -double.epsilon)
			{
				supports[1] = supports[3];
				outside = true; continue;
			}

			break;
		}
		size = 4;
		
		return 0;
	}

	int refinePortal(Shape!U A, Shape!U B)
	{
		vec3 dir;
		Sup v4;

		while(true)
		{
			dir = portalDirection;
			if(portalEncapsulatesOrigin(dir)) return 0;
			v4 = Sup(A, B, dir);
			if(!portalCanEncapsulateOrigin(v4, dir) || portalReachTolerance(v4, dir)) return -1;
			expandPortal(v4);
		}
	}

	void findPenetration(Shape!U A, Shape!U B, ref vec3 p1, ref vec3 p2)
	{
		uint loops;
		while(true)
		{
			vec3 dir = portalDirection;
			auto v4 = Sup(A, B, dir);

			if(portalReachTolerance(v4, dir) || loops > mprLoopMax)
			{
				vec3 pdir;
				double depth = sqrt(pointTriDistance2(vec3(), supports[1].p, supports[2].p, supports[3].p, pdir));
				if(pdir.isZero) pdir = dir;
				pdir.normalize;
				pdir *= depth * 0.5;
				findPos(A, B, p1, p2);
				p1 += p2;
				p1 *= 0.5;
				p2 = p1 + pdir;
				p1 -= pdir;
				return;

			}

			expandPortal(v4);
			loops++;
		}

	}

	void findPos(Shape!U A, Shape!U B, ref vec3 p1, ref vec3 p2)
	{
		double[4] b;
		b[0] = dot(cross(supports[1].p, supports[2].p), supports[3].p);
		b[1] = dot(cross(supports[3].p, supports[2].p), supports[0].p);
		b[2] = dot(cross(supports[0].p, supports[1].p), supports[3].p);
		b[3] = dot(cross(supports[2].p, supports[1].p), supports[0].p);
		double sum = b[0] + b[1] + b[2] + b[3];

		if(sum <= double.epsilon)
		{
			vec3 dir = portalDirection;
			b[0] = 0;
			b[1] = dot(cross(supports[2].p, supports[3].p), dir);
			b[2] = dot(cross(supports[3].p, supports[1].p), dir);
			b[3] = dot(cross(supports[1].p, supports[2].p), dir);
			sum = b[1] + b[2] + b[3];
		}

		double inv = 1.0 / sum;

		p1 = vec3.zero;
		p2 = vec3.zero;

		foreach(i; 0..4)
		{
			p1 += supports[i].Ap * b[i];
			p2 += supports[i].Bp * b[i];
		}

		p1 *= inv;
		p2 *= inv;
		//import std.experimental.logger;
		//std.experimental.logger.log(p1 - p2);
	}

	void expandPortal(ref Sup v4)
	{
		vec3 v4v0 = cross(v4.p, supports[0].p);

		if(0.0 < dot(supports[1].p, v4v0))
		{
			if(0.0 < dot(supports[2].p, v4v0)) supports[1] = v4;
			else supports[3] = v4;
		}
		else
		{
			if(0.0 < dot(supports[3].p, v4v0)) supports[2] = v4;
			else supports[1] = v4;
		}
	}

	vec3 portalDirection()
	{
		vec3 dir = cross(supports[2].p - supports[1].p, supports[3].p - supports[1].p);
		dir.normalize;
		return dir;
	}

	bool portalEncapsulatesOrigin(ref vec3 dir)
	{
		return dot(supports[1].p, dir) >= -double.epsilon;
	}

	bool portalReachTolerance(ref Sup v4, ref vec3 dir)
	{
		double dv1 = dot(supports[1].p, dir);
		double dv2 = dot(supports[2].p, dir);
		double dv3 = dot(supports[3].p, dir);
		double dv4 = dot(v4.p, dir);
		double mdot = fmin(fmin(dv4 - dv1, dv4 - dv2), dv4 - dv3);
		return mdot <= mprTolerance;
	}
	
	bool portalCanEncapsulateOrigin(ref Sup v4, ref vec3 dir)
	{
		return dot(v4.p, dir) > -double.epsilon;
	}
}

double pointSegDistance2(vec3 p, vec3 x0, vec3 b, ref vec3 witness)
{
	vec3 d = b - x0;
	vec3 a = x0 - p;
	double dist;

	double t = -dot(a, d);
	t /= d.lengthSquared;

	if(t <= double.epsilon)
	{
		dist = (x0 - p).lengthSquared;
		witness = x0;
	}
	else
	{
		if(t >= 1.0)
		{
			dist = (b - p).lengthSquared;
			witness = b;
		}
		else
		{
			witness = d*t + x0;
			dist = (witness - p).lengthSquared;
		}
	}

	return dist;
}

double pointTriDistance2(vec3 P_, vec3 x0, vec3 B, vec3 C, ref vec3 witness)
{
	vec3 d1 = B - x0;
	vec3 d2 = C - x0;
	vec3 a = x0 - P_;
	double dist;

	double u = dot(a, a);
	double v = dot(d1, d1);
	double w = dot(d2, d2);
	double p = dot(a, d1);
	double q = dot(a, d2);
	double r = dot(d1, d2);

	double s = (q * r - w * p) / (w * v - r * r);
	double t = (-s * r - q) / w;
	if(0.0 <= s && s <= 1.0 && 0.0 <= t && t <= 1.0 && t + s <= 1.0)
	{
		d1 *= s;
		d2 *= t;
		witness = x0;
		witness += d1;
		witness += d2;

		dist = (witness - P_).lengthSquared;
	}
	else
	{
		dist = pointSegDistance2(P_, x0, B, witness);
		vec3 witness2;
		double dist2 = pointSegDistance2(P_, x0, C, witness2);
		if(dist2 < dist)
		{
			dist = dist2;
			witness = witness2;
		}

		dist2 = pointSegDistance2(P_, B, C, witness2);
		if(dist2 < dist)
		{
			dist = dist2;
			witness = witness2;
		}
	}
	return dist;
}