public class Vec3 
{
	public Vec3()
	{
		x = y = z = 0;
	}
	
	public Vec3(float x, float y)
	{
		this.x = x;
		this.y = y;
		this.z = 0;
	}
	
	public Vec3(float x, float y, float z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public Vec3(Vec3 a)
	{
		x = a.x;
		y = a.y;
		z = a.z;
	}
	
	public void add(Vec3 a)
	{
		x += a.x;
		y += a.y;
		z += a.z;
	}
	
	public void scale(float s)
	{
		x *= s;
		y *= s;
		z *= s;
	}
	
	public float dist(Vec3 a)
	{
		float dx = x - a.x;
		float dy = y - a.y;
		float dz = z - a.z;
		return dx*dx + dy*dy + dz*dz;
	}
	
	public float x, y, z;
	
	public static Vec3 add(Vec3 a, Vec3 b)
	{
		return new Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	public static Vec3 add(Vec3 a, Vec3 b, Vec3 c)
	{
		return new Vec3(a.x + b.x + c.x, a.y + b.y + c.y, a.z + b.z + c.z);
	}

	public static Vec3 add(Vec3 a, Vec3 b, Vec3 c, Vec3 d)
	{
		return new Vec3(a.x + b.x + c.x + d.x, a.y + b.y + c.y + d.y, a.z + b.z + c.z + d.z);
	}
	
	public static Vec3 sub(Vec3 a, Vec3 b)
	{
		return new Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
	}
	
	public static Vec3 scale(float s, Vec3 a)
	{
		return new Vec3(s * a.x, s * a.y, s * a.z);
	}

	public static Vec3 scale(Vec3 a, float s)
	{
		return scale(s, a);
	}

	public static Vec3 div(Vec3 a, float d)
	{
		if (d == 0)
			return new Vec3(0,0,0);
		return scale(1/d, a);
	}
	
	public static Vec3 truncate(Vec3 v, Vec3 t)
	{
		float mx = v.x < 0 ? -1 : 1;
		float my = v.y < 0 ? -1 : 1;
		return new Vec3(
				Math.abs(v.x) > t.x ? mx * t.x : v.x,
				Math.abs(v.y) > t.y ? my * t.y : v.y);
	}
	
	public static Vec3 norm(Vec3 v)
	{
		float len = (float)Math.sqrt(v.x * v.x + v.y * v.y);
		if (len == 0)
			return new Vec3(0,0);
		return Vec3.div(v, len);
	}

}
