module raider.collision.shape;

import raider.math.all;
import raider.collision.space;
import raider.tools.reference;

package struct Plug(U)
{
	aabb3f aabb;
	P!(Shape!U) shape;
}

/**
 * Collision geometry.
 * 
 * Supported types:
 * - Sphere
 * - Box
 * - Convex hull
 * - Triangle soup
 * - Displacement map 
 */
final class Shape(U)
{private
	aabb3f aabb;
	Type type;

public:
	enum Type
	{
		Sphere,
		Box,
		ConvexHull,
		TriangleSoup,
		DisplacementMap,
		None
	}

	U data; //User data

	this() {}
	~this() { reset; }

	@property vec3 position() { return vec3(); }
	@property void position(vec3 value) { }
	@property mat3 orientation() { mat3 r; return r; }
	@property void orientation(mat3 value) { }

	private void reset()
	{
		type = Type.None;
	}
	
	void setSphere(double radius)
	{
		reset;
		type = Type.Sphere;
	}

	void setBox(vec3 dim)
	{
		reset;
		type = Type.Box;
	}

	void setConvexHull(vec3[] points)
	{
		reset;
		type = Type.ConvexHull;
	}
	
	void setTriangleSoup(vec3[] verts, uint[] tris)
	{
		reset;
		type = Type.TriangleSoup;
	}
	
	void setDisplacementMap(float[] values, uint width)
	{
		reset;
		type = Type.DisplacementMap;
	}
}