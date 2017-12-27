module raider.collision.shape;

import raider.math;
import raider.collision.space;
import raider.tools.reference;
import std.math : PI, abs;

immutable double featureRadius = 0.01;

/**
 * Collision geometry.
 * 
 * Supported types:
 * - Sphere
 * - Box
 * - Convex hull
 * 
 * TODO:
 * - Cylinder 
 * - Triangle soup 
 * - Displacement map 
 * - Automatic bevelling
 * 
 * 'U' template mixin parameter is for user data.
 * It will be instantiated as U data, and aliased to this.
 * 
 * Shapes can be modified during simulation.
 * 
 * A shape should not be used after its space is
 * destroyed.
 */
@RC abstract class Shape(U)
{package:
	alias Shape!U Shape_;
	alias Space!U Space_;

	//How do we deal with the global aabb? Fixed point?
	struct Proxy { aabb3 aabb; Shape_ shape; }

	W!Space_ space;
	//TODO Depending on algorithm structure it might be worth TRYING quats.
	//Are there any algorithms that require a matrix cache?
	//How about the support and feature functions? MMM?
	//Might have to profile the difference.
	//Besides, quat*point isn't THAT slow.. no cos or sin.
	//Also, multiplying by 2 becomes a shift.
	//..also I only see 13 multiplies in Eigen's source.
	//Unlike the 21 listed in Wikipedia's analysis.
	//You know, the main disadvantage of quaternions is starting to look like
	//less of a disadvantage. It's only when you have lots of vertex positions 
	//and non-affine transformations that matrices are absolutely required..

	@property auto proxy() //pretty sure this isn't a good idea, it does nothing useful
	{
		return Proxy(aabb, this);
	}

public:

	vec3 p;
	mat3f o = mat3f.identity;

	U data;
	//alias data this;

	this(Space_ space)
	{
		this.space = W!Space_(space);

		if(auto s = this.space._r)
			synchronized(s.mutex) s.shapes.add(proxy);
		//FIXME And if a collision callback wants to create more shapes?
		//near() adds contacts to lists, remember.
	}

	~this()
	{
		//Linear search to remove shape from space.
		//Can't do much better; the shapes array is continually changing.
		if(auto s = space._r)
			synchronized(s.mutex)
				s.shapes.destroyItem!"i.shape"(this);
	}

	//Subclasses implement these.
	aabb3 aabb();                    /// Local bounding box of shape.
	double volume();                 /// Volume of shape in metres cubed.
	mat3 inertiaLocal(ref vec3 com); /// Local inertia tensor and center of mass.
	vec3 supportLocal(ref vec3 dir); /// Local support function

	//@property vec3 position() { return p; }
	//@property void position(vec3 value) { p = value; }
	//@property mat3f orientation() { return o; }
	//@property void orientation(mat3f value) { o = value; }

	alias p pos, position;
	alias o ori, orientation;

	/**
	 * Find support point in global coordinates from a global direction
	 * 
	 * Unrolled for speed, this is an inner loop.
	 * TODO Sphere is a special case.
	 * Also TODO investigate Q16 implementation,
	 * if o is Q16.
	 */
	vec3 support(vec3 d)
	{
		//Get dir in local space and find support point
		vec3 s = void;
		s[0] = d[0] * o.f[0] + d[1] * o.f[3] + d[2] * o.f[6];
		s[1] = d[0] * o.f[1] + d[1] * o.f[4] + d[2] * o.f[7];
		s[2] = d[0] * o.f[2] + d[1] * o.f[5] + d[2] * o.f[8];
		s = supportLocal(s);

		//Return support point in global space (inc. translation)
		vec3 g = void;
		g[0] = s[0] * o.f[0] + s[1] * o.f[1] + s[2] * o.f[2];
		g[1] = s[0] * o.f[3] + s[1] * o.f[4] + s[2] * o.f[5];
		g[2] = s[0] * o.f[6] + s[1] * o.f[7] + s[2] * o.f[8];
		return g + p;
	}
}



@RC class Sphere(U) : Shape!U
{public:
	float radius;

	this(Space!U space, float radius = 1.0)
	{
		super(space);
		this.radius = radius;
	}

	override aabb3 aabb()
	{
		return aabb3(p - radius, p + radius);
	}

	override double volume()
	{
		return (4.0/3.0) * PI * radius*radius*radius;
	}

	override mat3 inertiaLocal(ref vec3 com)
	{
		return mat3.identity * 0.4 * radius;
	}

	override vec3 supportLocal(ref vec3 dir)
	{
		dir.normalize; dir *= radius; return dir;
	}
}

@RC class Box(U) : Shape!U
{public:
	vec3f dim;

	this(Space!U space, vec3 dimensions = vec3(1,1,1))
	{
		super(space);
		dim = dimensions.abs;
	}
	
	override aabb3 aabb()
	{
		vec3 aadim = vec3(dot(o[0].abs, dim), dot(o[1].abs, dim), dot(o[2].abs, dim));
		return aabb3(pos-aadim, pos + aadim);
		/* TODO Profile expanded box aabb generation
		 * aadim[0] = abs(o.f[0] * dim[0]) + abs(o.f[1] * dim[1]) + abs(o.f[2] * dim[2]);
		 * aadim[1] = abs(o.f[3] * dim[0]) + abs(o.f[4] * dim[1]) + abs(o.f[5] * dim[2]);
		 * aadim[2] = abs(o.f[6] * dim[0]) + abs(o.f[7] * dim[1]) + abs(o.f[8] * dim[2]);
		 * 
		 * Haha what is this hot mess
		 * aadim[0] = abs(o.f[0]) * dim[0] + abs(o.f[1]) * dim[1] + abs(o.f[2]) * dim[2];
		 * aadim[1] = abs(o.f[3]) * dim[0] + abs(o.f[4]) * dim[1] + abs(o.f[5]) * dim[2];
		 * aadim[2] = abs(o.f[6]) * dim[0] + abs(o.f[7]) * dim[1] + abs(o.f[8]) * dim[2];
		 */
		//WHAT'RE THOOOOOOSE
		//return aabb3f(pos - dim.length, pos + dim.length);
	}

	override double volume()
	{
		return dim[0] * dim[1] * dim[2];
	}

	override mat3 inertiaLocal(ref vec3 com)
	{
		vec3 a = dim * dim;
		mat3 r;
		r[0][0] = (a[1] + a[2]) / 12;
		r[1][1] = (a[0] + a[2]) / 12;
		r[2][2] = (a[0] + a[1]) / 12;
		return r;
	}
	
	override vec3 supportLocal(ref vec3 dir)
	{
		vec3 r = void;
		r[0] = dir[0] <= -0.0 ? -dim[0] : dim[0];
		r[1] = dir[1] <= -0.0 ? -dim[1] : dim[1];
		r[2] = dir[2] <= -0.0 ? -dim[2] : dim[2];
		return r;
	}
}

@RC class ConvexHull(U) : Shape!U
{
	this(Space!U space)
	{
		super(space);
	}
	
	Array!vec3f points; //A static Q0.16 array sounds good.
	//Some sort of super-dense winged edge representation.
	//Upper limit of 128 elements means one-byte indices.
	//Actually, taking the average of ONLY edge contributions gives vertex sharpness.
	//mmmbut a sphereish convex hull needs to have the same result. Smaller size affecting sharpness.
	//THE SUPPORT FUNCTION CAN DO THIS.. a few samples and BAM. 

	void updateHull()
	{
	/* Convex hull storage is like peeling an apple.
	 * One algorithm (updateHull) can take a pointcloud, sort the points,
	 * and 'wind' them in a certain order, eliminating interior points.
	 * The other algorithm (faces iterator) can take these wound points
	 * and efficiently reconstruct the tri-faces.
	 * 
	 * This might also update cached mass and volume properties.
	 */
	}



	override aabb3 aabb()
	{
		return aabb3();
	}

	override mat3 inertiaLocal(vec3 com)
	{
		/* Implementation adapted from code by Gero Mueller
		 * based on Mirtich, B 1996, 'Fast and Accurate 
		 * Computation of Polyhedral Mass Properties', 
		 * Journal of Graphics Tools, vol. 1, no. 2.
		 */
	}

	override vec3 supportLocal(ref vec3 dir)
	{
		return vec3();
	}
}

/++ These aren't convex..
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
	{ return vec3(); } //KD tree? BSP?

	override bool featureLocal(ref vec3 pos)
	{ return false; }
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
++/


/*
 * So hey, triangle soup hierarchy generation is similar
 * for meshes. Could low-detail physical and high-detail graphical
 * meshes operate with the same hierarchy?
 * 
 * Also, multi-draw-elements - is that capable of drawing
 * multiple selected bits of triangle soup..?
 */

/* 14-10-2017
 * Regarding feature awareness
 * 
 * A good hint for the sliding movement of persistent contacts is that
 * if one surface is a flat plane, and the other is a sharp point, the
 * contact should move with the sharp point - it is less likely to break.
 * Therefore, shapes provide featureLocal, a method that takes a contact
 * point in global space and returns a byte (0-127).
 * 
 * This value is the sharpness, where 0 is flat and 127 is sharp.
 * 
 * Within a sphere defined by featureRadius around the contact point,
 * assumed to lie on the shape surface (but not always projected to it 
 * accurately), a flat plane has sharpness 0, and an infinitesimal point 
 * has sharpness 127. featureRadius is a fairly arbitrary value that is
 * tweaked to play well with elastic contacts.
 * 
 * The sharpness is how much of the hemisphere below the contact normal
 * is empty. This definition is very vague, only used as a guide for
 * fast and simple sharpness calculations. There is no single solution;
 * this is about speed and the quality of the end result.
 * 
 * When a contact slides, it moves with the sharpest contact. This
 * maximises its lifespan. Once it breaks, a new contact replaces it.
 * 
 * Sharpness is represented as a byte because more precision is useless
 * and it means shapes can store explicit values in a smaller footprint.
 * 
 * Implementations may appreciate the 'ftni' (float to normalised integer)
 * function in raider.math.misc. 
 */

/* Alternatively we have separate traction and restitution springs.
 * Which might make it work better for wheels and such.
 * Simpler, too. Cheaper.
 * 
 * 
 */
