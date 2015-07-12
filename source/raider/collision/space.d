module raider.collision.space;

import raider.math.all;
import raider.tools.array;
import raider.tools.reference;
import raider.collision.shape;
import core.sync.mutex;

/**
 * Generates collision information for a list of shapes.
 * 
 * Optimised for environments where all objects are moving.
 * Parallel create, delete, update and testing (hopefully).
 * 
 * 'U' parameter is for userdata stored on shapes.
 */
final class Space(U)
{
package:
	alias Shape!U Shape_;
	struct Proxy { aabb3f aabb; P!Shape_ shape; }
	Mutex mutex;

public:
	Array!Proxy shapes, shapes1;

public:
	this()
	{
		//warning for posterity - synchronizing on
		//an uninitialized mutex causes.. confusion
		mutex = new Mutex();
	}

	~this()
	{
		assert(shapes.length == 0);
	}

	void collide(void function(Shape_ a, Shape_ b) near)
	{
		/*TODO Sort and Sweep and Stumble
		SAS-based non-incremental broadphase that
		avoids the overhead of incremental without being
		totally allergic to clustering or slow in sorting.

		Sorts with floating-point radix sort. A parallel 
		algorithm is possible (if not assuredly feasible).
		
		Detects clusters and 'stumbles' (defers them to a 
		perpendicular axis). Cluster detection is based on the 
		size of the overlap list. A cluster starts when it goes 
		past a threshold, and ends when it drops a certain 
		cumulative number of overlaps. If there are still too
		many overlaps, a new cluster begins immediately.

		All clusters are added to another SAS list and sorted
		in one go on a perpendicular axis. (Note, objects in
		more than one cluster are duplicated.)
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