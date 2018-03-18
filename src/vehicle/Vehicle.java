package vehicle;

import java.util.Optional;

public class Vehicle {
	
	public final double T = 1.2; //s
	public final double s_0 = 2.0; //m
	public class Pose {
		public double s;
		public double[] r;
		public double t;
		public double v;
		public double a;
		public Optional<Double> dv = Optional.empty();
		public Optional<Double> gap_a = Optional.empty();
		public double gap_d;
		public double lane;
		
		public Pose copy() {
			Pose p = new Pose();
			p.s = s;
			p.r = r.clone();
			p.t = t;
			p.v = v;
			p.a = a;
			p.dv = (dv.isPresent()) ? Optional.of(dv.get()) : Optional.empty();
			p.gap_a = (gap_a.isPresent()) ? Optional.of(gap_a.get()) : Optional.empty();
			p.gap_d = gap_d;
			p.lane = lane;
			return p;
		}
	}
	
	public double a_max = 5; // m/s^2
	public double v_max = 25; // m/s
	public double length = 4.5; // m
	
	public Vehicle front;
	public Vehicle back;
	public Vehicle leftFront;
	public Vehicle leftBack;
	public Vehicle rightFront;
	public Vehicle rightBack;
	
	public Pose pose;
	public Pose lastPose;
	
	public boolean used;
	
	public Vehicle(double lane) {
		this.lastPose = new Pose();
		this.lastPose.lane = lane;
		this.pose = lastPose;
	}
	public void update(double dt) {
		if (front == null) {
			pose.a = a_max * (1-lastPose.v/v_max);
			
		}
		else {
			pose.gap_d = Math.max(0, lastPose.v*T + lastPose.v*lastPose.dv.get() / a_max) + s_0;
			pose.a = a_max * (1-lastPose.v/v_max - pose.gap_d/lastPose.gap_a.get());
		}
		
		pose.v = lastPose.v + pose.a*dt;
		pose.s = lastPose.s + pose.v*dt + pose.a*dt*dt;

		if (pose.v < 0) {
			pose.v = 0;
			pose.s = lastPose.s;
		}
		
//		System.out.println(pose.s);
				
		lastPose = pose.copy();
		
	}
	
	public void updateDependent() {
		if (front != null && front.used == true) front = null;

		if (front == null) {
			pose.gap_a = Optional.empty();
			pose.dv = Optional.empty();
		}
		else {

			pose.gap_a = Optional.of(front.pose.s-pose.s-length);
			pose.dv = Optional.of(pose.v-front.pose.v);
		}
		
		lastPose = pose.copy();		
		
		
	}
	
	public double getS() {
		return pose.s;
	}
	public double[] getR() {
		return pose.r;
	}
	
}
