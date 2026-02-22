package GVF;

import java.util.ArrayList;

class PathSegment {
	public final Spline spline;
	public final boolean reversed, decel;
	public final double power;
	
	public PathSegment (Spline s, boolean rev, boolean dec, double pow) {
		spline = s;
		reversed = rev;
		decel = dec;
		power = pow;
	}
}

class PathData {
	public Vector2 velocity, acceleration;
	public double radius, power;
	public boolean reversed;
	
	public PathData (Vector2 vel, Vector2 accel, double r, double pow, boolean rev) {
		velocity = vel.clone();
		acceleration = accel.clone();
		radius = r;
		power = pow;
		reversed = rev;
	}
}

class GuidingVectors {
	public Vector2 tangential, pull, centripetal;
	public double tangentialMag;
	
	public GuidingVectors(Vector2 v_t, Vector2 v_p, Vector2 v_c) {
		tangential = v_t.clone();
		tangentialMag = tangential.mag();
		pull = v_p.clone();
		centripetal = v_c.clone();
	}
	
	public Vector2 theoreticalVel() { return Vector2.add(tangential, Vector2.add(pull, centripetal)); }
}

public class Path {
	private ArrayList <PathSegment> segments;
	private ArrayList <RepulsionPoint> repulsion;
	private Pose2d lastPose;
	
	public Path (Pose2d p, ArrayList <RepulsionPoint> rp) {
		segments = new ArrayList<>();
		repulsion = rp;
		lastPose = p.clone();
	}
	
	public Path addPoint(Pose2d p, boolean rev, boolean dec) {
		if (rev) p.heading += Math.PI;
		segments.add(new PathSegment (new Spline (lastPose, p), rev, dec, 1.0));
		lastPose = p.clone();
		return this;
	}
	
	public Path addPoint(Pose2d p, boolean rev, boolean dec, double pow) {
		if (rev) p.heading += Math.PI;
		segments.add(new PathSegment (new Spline (lastPose, p), rev, dec, pow));
		lastPose = p.clone();
		return this;
	}
	
	public static double k_p = 0.1666; // 1 / 6
	
	public GuidingVectors calculate(PathSegment currSegment, Pose2d robot) {
		Spline s = currSegment.spline;
		double tau = s.getT(robot);
		
		Vector2 v_t = s.getVel(tau);
		
		Vector2 v_p = new Vector2(s.getPos(tau).x - robot.x, s.getPos(tau).y - robot.y);
		v_p.mul(k_p);
		
		Vector2 v_r = new Vector2(0, 0);
		for(RepulsionPoint rp : repulsion) {
			v_r.add(rp.getInfluence(robot));
		}
		
		
		return new GuidingVectors(v_t, v_p, v_r);
	}
	
	public PathData update(Pose2d robot) {
		int index = 0;
		while (index < segments.size() && segments.get(index).spline.getT(robot) == 1.0) {
			index++;
		}
		
		if (index == segments.size()) return null;
		
		GuidingVectors current = calculate(segments.get(index), robot);
		GuidingVectors predict = calculate(segments.get(index), new Pose2d(robot.x + current.theoreticalVel().x * 0.001, robot.y + current.theoreticalVel().y * 0.001));
			
		/* 
		 * TODO: Contemplate the importance of tangential, whether or not it needs to be normed before adding. And if the original magnitude matters. This will influence final move vector!!! 
		 * (and also think about both methods [norm or mul by mag of tangential to pull and centripetal] if they will lead to teh same conclusion and the only diff is magnitude)
		 */
		
		return null;
	}
}
