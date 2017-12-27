module raider.collision.space;

import raider.math;
import raider.tools.array;
import raider.tools.reference;
import raider.collision.shape;
import core.sync.mutex;

/**
 * Stores shapes, finds collisions between them.
 * 
 * Optimised for environments where all objects are moving.
 * Parallel create, delete, update and testing (hopefully).
 * 
 * 'U' parameter is for userdata stored on shapes.
 */
@RC class Space(U)
{
package:
	alias Shape!U Shape_;
	alias Shape_.Proxy Proxy_;

	Mutex mutex;

public:
	Array!Proxy_ shapes, shapes1;

public:
	this()
	{
		//warning for posterity - synchronizing on
		//an uninitialized mutex causes.. confusion
		mutex = new Mutex();
	}

	~this()
	{
		synchronized(mutex)
			foreach(ref proxy; shapes)
				proxy.shape.space = null;
	}

	//TODO Pass function as a template parameter for static bind
	void collide(void function(Shape_ a, Shape_ b) near)
	{
		/*TODO Sort and Sweep and Stumble
		SAS-based non-incremental (one-shot) broadphase that
		avoids the overhead of incremental without being
		totally allergic to clustering or slow in sorting.

		Sorts either with floating-point radix sort or gnomesort.
		The AABBs are broken into a tree, with leaves that can be
		sorted separately. The tree is described with special tags
		stored within the stream of shape proxies.
		
		Due to complexity constraints, the tree is not dynamic, but
		specified opportunistically and artistically. Bodies in the
		physics engine will group their shapes, entities are expected
		to group their bodies (when appropriate), and other developer
		insights can create larger or more specific groupings.

		Radix sort is faster for a disordered state.
		What about one pass of radix on the MSB, then gnome? Reduces 
		the impact of massive disturbances, but we still get a little
		of the sweet, sweet almost-sorted performance.
		
		SASAS collision check:
		Detects clusters and 'stumbles' (defers them to a sweep 
		along a perpendicular axis). Cluster detection is based 
		on the size of the overlap list. A cluster starts when 
		it goes past a threshold, and ends when it drops a certain 
		cumulative number of overlaps. If there are still too
		many overlaps, a new cluster begins immediately.

		All clusters are added to another SAS list and sorted
		in one go on a perpendicular axis. (Note, proxies in
		more than one cluster are duplicated. Thus it is essential
		the proxy be as small as possible.)
		
		It might be appropriate to use fixed point AABBs.
		Floats and doubles both have logarithmic precision which
		is totally useless. A nice normalised long is much better.
		If the milky way is 100,000 light-years across, 2**64
		gives 51.2 distinct states per meter, or 19mm increments.
		Serviceable precision for AABBs in a space sim. It would
		be much worse with doubles.

		Plus, it makes radix sort simpler and faster.
		The dimensions of the AABB might be recorded as an offset,
		so they can be smaller. 

		For most applications, Q26.6 and Q10.6 seem appropriate - world size
		up to 67000km, AABB size up to 1km, 15mm precision.
		*/

		//Naive check
		if(!shapes.length) return; 

		//Proxies must be updated before the check
		foreach(x; 0..shapes.length) shapes[x].aabb = shapes[x].shape.aabb;

		for(uint i = 1; i < shapes.length; i++)
		{
			auto a = shapes[i];
			for(uint j = 0; j < i; j++)
			{
				auto b = shapes[j];
				if(a.aabb.intersects(b.aabb)) near(a.shape, b.shape);
			}
		}

		/*
		 * Tree sas: Only when the first element in the window is removed do we 
		 * tunnel into its internal collisions.
		 */
	}
}

unittest
{
	import std.stdio;
	import raider.collision.shape;
	
	auto space = New!(Space!int)();

	{
		auto a = New!(Sphere!int)(space, 1.0);
		auto b = New!(Sphere!int)(space, 1.0);
		auto c = New!(Sphere!int)(space, 1.0);
		a.data = 0;
		b.data = 0;
		c.data = 0;
		a.pos = vec3(0,1,0);
		b.pos = vec3(0,0,0);
		c.pos = vec3(0,0,0);

		space.collide(function void(Shape!int a, Shape!int b) { a.data++; b.data++; });
		assert(a.data == 2);
		assert(b.data == 2);
		assert(c.data == 2);
	}
}


/* 11-8-2017
 * Regarding options..
 * Most engines have a compile-time option to select between float or double precision.
 * Because of our focus on memory footprint (and its influence on overall performance),
 * we want to allow greater control, including offering fixed-point math, which is good
 * for less capable instruction sets, and is usually more appropriate for physics.
 * 
 * Rotations: Quat or matrix, double/float/Q0.16/Q0.32
 * Positions: double/float/Q?.?
 * AABBs: double/float/Q26.6-Q10.6/??
 * 
 */
