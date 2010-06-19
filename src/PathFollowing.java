import java.util.ArrayList;
import processing.core.PApplet;

public class PathFollowing extends PApplet 
{
	// an implementation of Seek steering behavior
	// http://www.red3d.com/cwr/steer/SeekFlee.html

	Vehicle vehicle = new Vehicle(new Vec3(200, 200), new Vec3(0, 0), this);
	//Vec3 target = new Vec3(150,300);
	int starting_pt = 0;
	float max_speed = 5;
	float closest_s = -1;
	
	ArrayList<Vec3> points = new ArrayList<Vec3>();
	float[] steps_newton;
	float[] steps_quadratic;
	
	Vec3 h(float scale, float[] coeffs, Vec3[] pts)
	{
		Vec3 res = new Vec3(0, 0, 0);
		for (int i = 0; i < pts.length; ++i) {
			res.add(Vec3.scale(coeffs[i], pts[i]));
		}
		return Vec3.scale(scale, res);
	}
	
	
	Vec3 catmull(float t, Vec3 p1, Vec3 p2, Vec3 p3, Vec3 p4)
	{
		//  q(t) = 0.5 * 
		// ((-p1 + 3*p2 -3*p3 + p4)*t*t*t
        // + (2*p1 -5*p2 + 4*p3 - p4)*t*t
		// + (-p1+p3)*t
		// + 2*p2)
		float t2 = t*t;
		float t3 = t2*t;
		
		Vec3[] pts = {p1, p2, p3, p4};
		float[] a = {+0, +2, +0, +0};
		float[] b = {-1, +0, +1, +0};
		float[] c = {+2, -5, +4, -1};
		float[] d = {-1, +3, -3, +1};
		
		return Vec3.add(
				h(0.5f * 1, a, pts),
				h(0.5f * t, b, pts),
				h(0.5f * t2, c, pts),
				h(0.5f * t3, d, pts));
	}

	Vec3 catmull_vel(float t, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		float t2 = t*t;
		//float t3 = t2*t;
		
		Vec3[] pts = {p0, p1, p2, p3};
		//float[] a = {+0, +2, +0, +0};
		float[] b = {-1, +0, +1, +0};
		float[] c = {+2, -5, +4, -1};
		float[] d = {-1, +3, -3, +1};
		
		return Vec3.add(
				h(0.5f * 1, b, pts),
				h(0.5f * 2 * t, c, pts),
				h(0.5f * 3 * t2, d, pts));
	}

	Vec3 catmull_acc(float t, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		//float t2 = t*t;
		//float t3 = t2*t;
		
		Vec3[] pts = {p0, p1, p2, p3};
		//float[] a = {+0, +2, +0, +0};
		//float[] b = {-1, +0, +1, +0};
		float[] c = {+2, -5, +4, -1};
		float[] d = {-1, +3, -3, +1};
		
		return Vec3.add(
				h(0.5f * 2, c, pts),
				h(0.5f * 6 * t, d, pts));
	}
	
	float d_prim(float s, Vec3 x0, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		float s2 = s * s;
		float s3 = s2 * s;
		return 
			1.0f*(3*(p3.x-3*p2.x+3*p1.x-p0.x)*s2+2*(-p3.x+4*p2.x-5*p1.x+2*p0.x)*s+p2.x-p0.x)*(0.5f*((p3.x-3*p2.x+3*p1.x-p0.x)*s3+(-p3.x+4*p2.x-5*p1.x+2*p0.x)*s2+(p2.x-p0.x)*s+2*p1.x)-x0.x) +
			1.0f*(3*(p3.y-3*p2.y+3*p1.y-p0.y)*s2+2*(-p3.y+4*p2.y-5*p1.y+2*p0.y)*s+p2.y-p0.y)*(0.5f*((p3.y-3*p2.y+3*p1.y-p0.y)*s3+(-p3.y+4*p2.y-5*p1.y+2*p0.y)*s2+(p2.y-p0.y)*s+2*p1.y)-x0.y) +
			1.0f*(3*(p3.z-3*p2.z+3*p1.z-p0.z)*s2+2*(-p3.z+4*p2.z-5*p1.z+2*p0.z)*s+p2.z-p0.z)*(0.5f*((p3.z-3*p2.z+3*p1.z-p0.z)*s3+(-p3.z+4*p2.z-5*p1.z+2*p0.z)*s2+(p2.z-p0.z)*s+2*p1.z)-x0.z);
	}
	
	float d_prim_prim(float s, Vec3 x0, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		float s2 = s * s;
		float s3 = s2 * s;
		float tmpx = (3*(p3.x-3*p2.x+3*p1.x-p0.x)*s2+2*(-p3.x+4*p2.x-5*p1.x+2*p0.x)*s+p2.x-p0.x);
		float tmpy = (3*(p3.y-3*p2.y+3*p1.y-p0.y)*s2+2*(-p3.y+4*p2.y-5*p1.y+2*p0.y)*s+p2.y-p0.y);
		float tmpz = (3*(p3.z-3*p2.z+3*p1.z-p0.z)*s2+2*(-p3.z+4*p2.z-5*p1.z+2*p0.z)*s+p2.z-p0.z);
		return 
			1.0f*(6*(p3.x-3*p2.x+3*p1.x-p0.x)*s+2*(-p3.x+4*p2.x-5*p1.x+2*p0.x))*(0.5f*((p3.x-3*p2.x+3*p1.x-p0.x)*s3+(-p3.x+4*p2.x-5*p1.x+2*p0.x)*s2+(p2.x-p0.x)*s+2*p1.x)-x0.x)+0.5f * tmpx * tmpx +
			1.0f*(6*(p3.y-3*p2.y+3*p1.y-p0.y)*s+2*(-p3.y+4*p2.y-5*p1.y+2*p0.y))*(0.5f*((p3.y-3*p2.y+3*p1.y-p0.y)*s3+(-p3.y+4*p2.y-5*p1.y+2*p0.y)*s2+(p2.y-p0.y)*s+2*p1.y)-x0.y)+0.5f * tmpy * tmpy +
			1.0f*(6*(p3.z-3*p2.z+3*p1.z-p0.z)*s+2*(-p3.z+4*p2.z-5*p1.z+2*p0.z))*(0.5f*((p3.z-3*p2.z+3*p1.z-p0.z)*s3+(-p3.z+4*p2.z-5*p1.z+2*p0.z)*s2+(p2.z-p0.z)*s+2*p1.z)-x0.z)+0.5f * tmpz * tmpz;
	}
	
	float d_(float s, Vec3 x0, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		// D'(s) = (0.5*((p3-3*p2+3*p1-p0)*s^3+(-p3+4*p2-5*p1+2*p0)*s^2+(p2-p0)*s+2*p1)-x0)^2
		float s2 = s * s;
		float s3 = s2 * s;
		float tmpx = (0.5f*((p3.x-3*p2.x+3*p1.x-p0.x)*s3+(-p3.x+4*p2.x-5*p1.x+2*p0.x)*s2+(p2.x-p0.x)*s+2*p1.x)-x0.x); 
		float tmpy = (0.5f*((p3.y-3*p2.y+3*p1.y-p0.y)*s3+(-p3.y+4*p2.y-5*p1.y+2*p0.y)*s2+(p2.y-p0.y)*s+2*p1.y)-x0.y); 
		float tmpz = (0.5f*((p3.z-3*p2.z+3*p1.z-p0.z)*s3+(-p3.z+4*p2.z-5*p1.z+2*p0.z)*s2+(p2.z-p0.z)*s+2*p1.z)-x0.z);
		return tmpx * tmpx + tmpy * tmpy + tmpz * tmpz; 
	}
	
	int safe_idx(int idx)
	{
		return Math.min(points.size()-1, Math.max(0, idx));
	}
	
	float minimize_newton(int iterations, float initial_guess, Vec3 target_pt)
	{
		// the function we want to minimize is:
		// D(s) = (x(s) - x0)^2 + (y(s) - y0)^2 + (z(s) - z0)^2
		
		// Newton's method gives us:
		// s[m+1] = s[m] - D'(s) / D''(s)
		
		// D'(s) = (0.5*((p3-3*p2+3*p1-p0)*s^3+(-p3+4*p2-5*p1+2*p0)*s^2+(p2-p0)*s+2*p1)-x0)^2
		
		// D'(t)  = 1.0*(3*(p3-3*p2+3*p1-p0)*s^2+2*(-p3+4*p2-5*p1+2*p0)*s+p2-p0)*(0.5*((p3-3*p2+3*p1-p0)*s^3+(-p3+4*p2-5*p1+2*p0)*s^2+(p2-p0)*s+2*p1)-x0)
		// D''(t) = 1.0*(6*(p3-3*p2+3*p1-p0)*s+2*(-p3+4*p2-5*p1+2*p0))*(0.5*((p3-3*p2+3*p1-p0)*s^3+(-p3+4*p2-5*p1+2*p0)*s^2+(p2-p0)*s+2*p1)-x0)+0.5*(3*(p3-3*p2+3*p1-p0)*s^2+2*(-p3+4*p2-5*p1+2*p0)*s+p2-p0)^2
		// 

		Vec3[] pts = new Vec3[points.size()];
		points.toArray(pts);

		steps_newton = new float[iterations];
		float s = initial_guess;
		Vec3 p0 = pts[safe_idx(starting_pt - 1)];
		Vec3 p1 = pts[safe_idx(starting_pt - 0)];
		Vec3 p2 = pts[safe_idx(starting_pt + 1)];
		Vec3 p3 = pts[safe_idx(starting_pt + 2)];
		for (int i = 0; i < iterations; ++i) {
			s = s - d_prim(s, target_pt, p0, p1, p2, p3) / d_prim_prim(s, target_pt, p0, p1, p2, p3);
			steps_newton[i] = s;
		}
		return s;
	}
	
	float pee(float s, float s1, float s2, float s3, Vec3 x0, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		return 
		((s-s2)*(s-s3)) / ((s1-s2)*(s1-s3)) * d_(s1, x0, p0, p1, p2, p3) +
		((s-s1)*(s-s3)) / ((s2-s1)*(s2-s3)) * d_(s2, x0, p0, p1, p2, p3) +
		((s-s1)*(s-s2)) / ((s3-s1)*(s3-s2)) * d_(s3, x0, p0, p1, p2, p3);
	}
	
	int max_idx(float x0, float x1, float x2, float x3)
	{
		int idx = 0;
		float max_value = x0;
		if (x1 > max_value) { idx = 1; max_value = x1; }
		if (x2 > max_value) { idx = 2; max_value = x2; }
		if (x3 > max_value) { idx = 3; max_value = x3; }
		return idx;
	}
	
	float minimize_quadratic(int iterations, Vec3 target_pt)
	{
		// initial values
		float s1 = 0;
		float s2 = 0.5f;
		float s3 = 1;
		
		Vec3[] pts = new Vec3[points.size()];
		points.toArray(pts);

		Vec3 pp0 = pts[safe_idx(starting_pt-1)];
		Vec3 pp1 = pts[safe_idx(starting_pt-0)];
		Vec3 pp2 = pts[safe_idx(starting_pt+1)];
		Vec3 pp3 = pts[safe_idx(starting_pt+2)];

		float s = 0;
	
		steps_quadratic = new float[iterations];
		for (int i = 0; i < iterations; i++) {
		
			float y12 = s1*s1 - s2*s2;
			float y23 = s2*s2 - s3*s3;
			float y31 = s3*s3 - s1*s1;
			
			float s12 = s1 - s2;
			float s23 = s2 - s3;
			float s31 = s3 - s1;

			float d1 = d_(s1, target_pt, pp0, pp1, pp2, pp3); 
			float d2 = d_(s2, target_pt, pp0, pp1, pp2, pp3); 
			float d3 = d_(s3, target_pt, pp0, pp1, pp2, pp3);

			// new value for s
			s = 0.5f * (y23*d1 + y31*d2 + y12*d3) / (s23*d1 + s31*d2 + s12*d3);
			steps_quadratic[i] = s;
			
			// calc values for p(s) with the 4 s values, and keep 3 lowest 
			float p0 = pee(s, s1, s2, s3, target_pt, pp0, pp1, pp2, pp3);
			float p1 = pee(s1, s1, s2, s3, target_pt, pp0, pp1, pp2, pp3);
			float p2 = pee(s2, s1, s2, s3, target_pt, pp0, pp1, pp2, pp3);
			float p3 = pee(s3, s1, s2, s3, target_pt, pp0, pp1, pp2, pp3);

			// discard the largest value
			int idx = max_idx(p0, p1, p2, p3);
			switch (idx) {
			case 0: break;
			case 1: s1 = s; break;
			case 2: s2 = s; break;
			case 3: s3 = s; break;
			}
		}
	
		return s;
	}		

	float minimize(Vec3 target_pt)
	{
		float quad_s = minimize_quadratic(4, target_pt);
		float newton_s = minimize_newton(4, quad_s, target_pt);
		return newton_s;
	}

	public void setup()
	{
		size(400, 400);
		points.add(new Vec3(0, 0));
		points.add(new Vec3(0, 0));

		points.add(new Vec3(100, 100));
		points.add(new Vec3(150, 150));
		
		points.add(new Vec3(230, 300));
		points.add(new Vec3(330, 100));
		points.add(new Vec3(350, 50));
		points.add(new Vec3(350, 50));
	}
	
	public void mouseClicked() 
	{
		Vec3[] pts = new Vec3[points.size()];
		points.toArray(pts);

		Vec3 pt = new Vec3(mouseX, mouseY, 0);
		int cur = 0;
		float dist = pts[1].dist(pt);
		
		// find the closest control point
		for (int i = 1; i < pts.length; ++i) {
			float cand = pts[i].dist(pt);
			if (cand  < dist) {
				dist = cand;
				cur = i;
			}
		}

		Vec3 tmp = new Vec3(mouseX, mouseY);

		// try the two segments that share the closest control point
		if (cur == 0 || cur == pts.length - 1) {
			// only 1 point
			closest_s = minimize(tmp);
		} else {
			starting_pt = cur-1;
			float t1 = minimize(tmp);
			float cand1 = catmull(t1, pts[safe_idx(starting_pt-1)], pts[safe_idx(starting_pt-0)], pts[safe_idx(starting_pt+1)], pts[safe_idx(starting_pt+2)]).dist(tmp);
			starting_pt = cur;
			float t2 = minimize(tmp);
			float cand2 = catmull(t2, pts[safe_idx(starting_pt-1)], pts[safe_idx(starting_pt-0)], pts[safe_idx(starting_pt+1)], pts[safe_idx(starting_pt+2)]).dist(tmp);
			
			if (Math.abs(cand1) < Math.abs(cand2)) {
				starting_pt = cur-1;
				closest_s = t1;
			} else {
				starting_pt = cur;
				closest_s = t2;
			}
		}
	}
	
	void line(Vec3 p0, Vec3 p1)
	{
		line(p0.x, p0.y, p1.x, p1.y);
	}
	
	void draw_iterations()
	{
		Vec3[] pts = new Vec3[points.size()];
		points.toArray(pts);

		if (steps_newton != null) {
			stroke(0, 0, 0);
			fill(125, 125, 0);
			for (int i = 0; i < 4; ++i) {
				Vec3 p0x = catmull(steps_newton[i], pts[0], pts[1], pts[2], pts[3]);
				ellipse(p0x.x, p0x.y, 5, 5);
			}
		}

		if (steps_quadratic != null) {
			fill(125, 0, 125);
			for (int i = 0; i < 4; ++i) {
				Vec3 p0x = catmull(steps_quadratic[i], pts[0], pts[1], pts[2], pts[3]);
				ellipse(p0x.x, p0x.y, 5, 5);
			}
		}
		
		Vec3 p0x = catmull(closest_s, pts[safe_idx(starting_pt-1)], pts[safe_idx(starting_pt+0)], pts[safe_idx(starting_pt+1)], pts[safe_idx(starting_pt+2)]);
		stroke(0, 0, 0);
		fill(250, 250, 0);
		ellipse(p0x.x, p0x.y, 5, 5);

	}
		
	public void draw()
	{
		background(175, 200, 175);

		Vec3[] pts = new Vec3[points.size()];
		points.toArray(pts);

		noFill();
		int steps = 5;
		for (int i = 1; i < points.size()-2; i++) {
			for (int j = 0; j < steps; ++j) {
				float t0 = (float)(j+0)/(steps);
				float t1 = (float)(j+1)/(steps);
				stroke(0, 0, 0);
				// pos
				Vec3 p0 = catmull(t0, pts[i-1], pts[i], pts[i+1], pts[i+2]);
				Vec3 p1 = catmull(t1, pts[i-1], pts[i], pts[i+1], pts[i+2]); 
				line(p0, p1);
			}
		}

/*		
		// draw target
		stroke(0, 0, 0);
		fill(200, 200, 0);
		ellipse(target.x, target.y, 15, 15);
*/
		// draw vehicle
		stroke(0, 0, 0);
		fill(0, 200, 0);
		ellipse(vehicle.pos.x, vehicle.pos.y, 20, 20);

		// draw dir
		stroke(150, 150, 0);
		noFill();
		vehicle.DrawVel(40);
		
		// calc distance from vehicle position to closest point on spline
		//float dist = 
/*		
		Vec3 desired_velocity = Vec3.norm(Vec3.sub(target, vehicle.pos));
		vehicle.steering_dir = Vec3.sub(desired_velocity, vehicle.vel);
*/		
		// move vehicle
		vehicle.Integrate();
	}
}
