import processing.core.*;

public class Vehicle 
{
	public Vehicle(Vec3  p, Vec3 v, PApplet app)
	{
		pos = p;
		vel = v;
		this.app = app;
	}
	
	public void WrapPos()
	{
		if (pos.x > app.width) pos.x = 0;
		if (pos.x < 0) pos.x = app.width;
		if (pos.y > app.height) pos.y = 0;
		if (pos.y < 0) pos.y = app.height;
	}
	
	public void Integrate()
	{
		Vec3 steering_force = Vec3.truncate(steering_dir, max_force);
		// F = m * a => a = F / m
		Vec3 acc = Vec3.div(steering_force, mass);
		vel = Vec3.truncate(Vec3.add(vel, acc), max_vel);
		pos = Vec3.add(pos, vel);
		
		WrapPos();
	}
	
	public void DrawVel(float scale)
	{
		app.line(pos.x, pos.y, pos.x + scale * vel.x, pos.y + scale * vel.y);

	}

	Vec3 max_force = new Vec3(1, 1);
	Vec3 max_vel = new Vec3(1, 1);

	PApplet app;
	public Vec3 pos = new Vec3(0,0);
	public Vec3 vel = new Vec3(0,0);
	public Vec3 steering_dir = new Vec3(0,0);
	public float mass = 100;
}
