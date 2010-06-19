import processing.core.PApplet;

public class Seek extends PApplet 
{
	// an implementation of Seek steering behavior
	// http://www.red3d.com/cwr/steer/SeekFlee.html

	Vehicle vehicle = new Vehicle(new Vec3(200, 200), new Vec3(0, 0), this);
	public Vec3 target = new Vec3(150,300);
	float max_speed = 5;

	public void setup()
	{
		size(400, 400);
	}
	
	public void mouseClicked() 
	{
		target.x = mouseX;
		target.y = mouseY;
	}
		
	public void draw()
	{
		background(175, 200, 175);

		// draw target
		stroke(0, 0, 0);
		fill(200, 200, 0);
		ellipse(target.x, target.y, 15, 15);

		// draw vehicle
		stroke(0, 0, 0);
		fill(0, 200, 0);
		ellipse(vehicle.pos.x, vehicle.pos.y, 20, 20);

		// draw dir
		stroke(150, 150, 0);
		noFill();
		vehicle.DrawVel(40);
		
		Vec3 desired_velocity = Vec3.norm(Vec3.sub(target, vehicle.pos));
		vehicle.steering_dir = Vec3.sub(desired_velocity, vehicle.vel);
		
		// move vehicle
		vehicle.Integrate();
	}
}
