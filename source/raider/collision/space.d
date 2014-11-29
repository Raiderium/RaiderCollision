module raider.collision.space;

import raider.tools.array;
import raider.tools.reference;
import raider.collision.shape;

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
	alias Shape!U Shape_;
	alias Plug!U Plug_;
	Array!Plug_ plugs, plugs2;
	Array!(R!Shape_) shapes;

	void collide(void function(P!Shape_ a, P!Shape_ b) near)
	{
		//Naive check to allow development of dependent code.
		if(!plugs.length) return; 

		for(uint i = 1; i < plugs.length; i++)
		{
			Plug_ a = plugs[i];
			for(uint j = 0; j < i; j++)
			{
				Plug_ b = plugs[j];
				if(a.aabb.intersects(b.aabb)) near(a.shape, b.shape);
			}
		}

		/*TODO Sort and Sweep and Stumble
		Possible 1D-SAS-based non-incremental broadphase that
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
	}
}

/**
 * Floating point radix sort
 * 
 * Implementation based on http://stereopsis.com/radix.html
 */
private void radixSortFloats(float* input, float* sorted, uint elements)
{
	uint* sort = cast(uint*)sorted; 
	uint* array = cast(uint*)input;
	
	immutable uint kHist = 2048;
	uint b1Array[kHist * 3];
	
	uint* b0 = b1Array.ptr;
	uint* b1 = b0 + kHist;
	uint* b2 = b1 + kHist;
	
	for(int x = 0; x < elements; x++)
	{
		uint fi = array[x];
		
		//Flip float
		int m = fi >> 31;
		fi ^= -m | 0x80000000;
		
		b0[fi & 0x7FF]++;
		b1[fi >> 11 & 0x7FF]++;
		b2[fi >> 22]++;
	}
	
	uint s0, s1, s2, st;
	
	for(int x = 0; x < kHist; x++)
	{ st = b0[x] + s0; b0[x] = s0 - 1; s0 = st; }
	
	for(int x = 0; x < kHist; x++)
	{ st = b1[x] + s1; b1[x] = s1 - 1; s1 = st; }
	
	for(int x = 0; x < kHist; x++)
	{ st = b2[x] + s2; b2[x] = s2 - 1; s2 = st; }
	
	for(int x = 0; x < elements; x++)
	{
		uint fi = array[x];
		
		//Flip float
		int m = fi >> 31;
		fi ^= -m | 0x80000000;
		
		uint pos = fi & 0x7FF;
		sort[++b0[pos]] = fi;
	}
	
	for(int x = 0; x < elements; x++)
	{
		uint si = sort[x];
		uint pos = si >> 11 & 0x7FF;
		array[++b1[pos]] = si;
	}
	
	for(int x = 0; x < elements; x++)
	{
		uint ai = array[x];
		uint pos = ai >> 22;
		
		//Unflip float
		uint m = ((ai >> 31) - 1) | 0x80000000;	
		ai ^= m;
		sort[++b2[pos]] = ai;
	}
}