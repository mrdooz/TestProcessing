import processing.core.*;

public class Wander extends PApplet 
{
	// an implementation of Wander steering behavior
	// http://www.red3d.com/cwr/steer/Wander.html

	float angle_x = 0;
	Vehicle vehicle = new Vehicle(new Vec3(200, 200), new Vec3(0, 0), this);

	public void setup()
	{
		size(400, 400);
		angle_x = PI/2;
	}

	public void draw()
	{
		background(175, 200, 175);

		// draw vehicle
		stroke(0, 0, 0);
		fill(0, 200, 0);
		ellipse(vehicle.pos.x, vehicle.pos.y, 20, 20);

		// draw dir
		stroke(150, 150, 0);
		noFill();
		vehicle.DrawVel(40);

		// draw wander circle
		stroke(0, 0, 0);
		noFill();
		float wander_diameter = 50;
		Vec3 dir = Vec3.norm(vehicle.vel);
		Vec3 center = Vec3.add(vehicle.pos, Vec3.scale(40, dir));
		ellipse(center.x, center.y, wander_diameter, wander_diameter);

		// draw wander direction
		noStroke();
		fill(230, 0, 0);
		float wx = cos(angle_x), wy = sin(angle_x);
		float wander_x = center.x + wander_diameter/2 * wx;
		float wander_y = center.y + wander_diameter/2 * wy;
		ellipse(wander_x, wander_y, 5, 5);

		stroke(0, 100, 100);
		noFill();
		line(vehicle.pos.x, vehicle.pos.y, wander_x, wander_y);

		vehicle.steering_dir.x = wx;
		vehicle.steering_dir.y = wy;
		
		// move vehicle
		vehicle.Integrate();

		float r = 0.2f;
		angle_x += random(-r, r);
	}
}
