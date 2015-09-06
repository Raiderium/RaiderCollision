module raider.collision.shape;

import raider.math;
import raider.collision.space;
import raider.tools.reference;
import std.math;

/**
 * Collision geometry.
 * 
 * Supported types:
 * - Sphere
 * - Box
 * - Convex hull
 * 
 * TODO:
 * - Triangle soup 
 * - Displacement map 
 * 
 * 'U' parameter is for userdata.
 * 
 * Shapes can be modified during simulation.
 * 
 * A shape should not outlive its space. If it does, the space
 * destructor will assert.
 */
abstract class Shape(U)
{private:
	alias Space!U Space_;
	alias Shape!U Shape_;
	
	P!Space_ space;
	vec3 _pos;
	mat3 _ori;

	@property Space_.Proxy proxy()
	{
		return Space_.Proxy(aabb, P!Shape_(this));
	}

public:
	U data;

	this(Space_ space)
	{
		this.space = P!Space_(space);
		_pos = vec3(0,0,0);
		_ori = mat3.identity;

		synchronized(space.mutex)
		{
			space.shapes.add(proxy);
		}
	}

	~this()
	{
		//Linear search to remove shape from space.
		//Can't do much better; the shapes array is continually changing.
		synchronized(space.mutex)
		{
			foreach(x; 0..space.shapes.size)
			{
				if(space.shapes[x].shape == this)
				{
					space.shapes.removeFast(x);
					return;
				}
			}
			assert(0);
		}
	}

	aabb3f aabb();
	vec3 supportLocal(ref vec3 dir);

	@property vec3 pos() { return _pos; }
	@property void pos(vec3 value) { _pos = value; }
	@property mat3 ori() { return _ori; }
	@property void ori(mat3 value) { _ori = value; }

	///Finds support point in global coordinates
	//Unrolled for speed, this is an inner loop
	vec3 support(vec3 dir)
	{
		//Get dir in local space and find support point
		vec3 p = void;
		p[0] = dir[0] * _ori.f[0] + dir[1] * _ori.f[3] + dir[2] * _ori.f[6];
		p[1] = dir[0] * _ori.f[1] + dir[1] * _ori.f[4] + dir[2] * _ori.f[7];
		p[2] = dir[0] * _ori.f[2] + dir[1] * _ori.f[5] + dir[2] * _ori.f[8];
		p = supportLocal(p);

		//Return support point in global space (inc. translation)
		vec3 g = void;
		g[0] = p[0] * _ori.f[0] + p[1] * _ori.f[1] + p[2] * _ori.f[2];
		g[1] = p[0] * _ori.f[3] + p[1] * _ori.f[4] + p[2] * _ori.f[5];
		g[2] = p[0] * _ori.f[6] + p[1] * _ori.f[7] + p[2] * _ori.f[8];
		return g + _pos;
	}
}

class Sphere(U) : Shape!U
{public:
	double radius;

	this(Space!U space, double radius = 1.0)
	{
		super(space);
		this.radius = radius;
	}

	override aabb3f aabb()
	{
		return aabb3f(pos - radius, pos + radius);
	}

	override vec3 supportLocal(ref vec3 dir)
	{
		dir.normalize; dir *= radius; return dir;
	}
}

class Box(U) : Shape!U
{public:
	vec3 dim;

	this(Space!U space, vec3 dimensions = vec3(1,1,1))
	{
		super(space);
		dim = dimensions.abs;
	}
	
	override aabb3f aabb()
	{
		vec3 aadim = void;
		double* o = _ori.ptr;
		aadim[0] = 0.5f * (abs(o[0] * dim[0]) + abs(o[1] * dim[1]) + abs(o[2] * dim[2]));
		aadim[1] = 0.5f * (abs(o[3] * dim[0]) + abs(o[4] * dim[1]) + abs(o[5] * dim[2]));
		aadim[2] = 0.5f * (abs(o[6] * dim[0]) + abs(o[7] * dim[1]) + abs(o[8] * dim[2]));
		return aabb3f(pos-aadim, pos + aadim);
	}
	
	override vec3 supportLocal(ref vec3 dir)
	{
		vec3 r = void;
		foreach(x; 0..3) r[x] = (dir[x] <= -0.0 ? -1.0 : 1.0) * dim[x];
		return r;
	}
}

class ConvexHull(U) : Shape!U
{
	this(Space!U space)
	{
		super(space);
	}
	
	Array!vec3 points;

	override aabb3f aabb()
	{
		return aabb3f();
	}

	override vec3 supportLocal(ref vec3 dir)
	{
		return vec3();
	}
}

class TriangleSoup(U) : Shape!U
{
	this(Space!U space)
	{
		super(space);
	}
	
	Array!vec3 verts;
	Array!uint tris;

	override aabb3f aabb()
	{
		return aabb3f();
	}

	override vec3 supportLocal(ref vec3 dir)
	{ return vec3(); }
}

class DisplacementMap(U) : Shape!U
{
	this(Space!U space)
	{
		super(space);
	}
	
	Array!float values;
	uint width;

	override aabb3f aabb()
	{
		return aabb3f();
	}

	override vec3 supportLocal(ref vec3 dir)
	{ return vec3(); }
}