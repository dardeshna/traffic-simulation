package vehicle;

import java.util.ArrayList;
import java.util.Optional;

import road.Points;
import road.Road;

public class Vehicle {
	
	public static int total = 0;
	public int ID;
	public boolean color = false;
	
	public class Pose {
		
		private Pose() { }
		
		public Pose(double lane, double lane_count) {
			this.lane = lane;
			if (lane == lane_count-1) s_right = Optional.empty();
			if (lane == 0) s_left = Optional.empty();
		}
		
		public double s;  //Arc length position
		public Optional<Double> s_left = Optional.of(0.0);  //Arc length position
		public Optional<Double> s_right = Optional.of(0.0);  //Arc length position

		public double[] r;  //2D position
		public double t;  //Parametric position
		public double v;  //Velocity
		public double a;  //Acceleration
		public Optional<Double> gap_d;  //Desired gap with car in front
		public double lane;  //Lane (lateral position)
		
		public Pose copy() {
			Pose p = new Pose();
			p.s = s;
			p.s_left = (s_left.isPresent()) ? Optional.of(s_left.get()) : Optional.empty();
			p.s_right = (s_right.isPresent()) ? Optional.of(s_right.get()) : Optional.empty();
			p.r = r.clone();
			p.t = t;
			p.v = v;
			p.a = a;
			p.gap_d = gap_d;
			p.lane = lane;
			return p;
		}
	}
	
	public double T = 1.2; //s, desired time gap between cars
	public double s_0 = 2.0; //m, minimum spacing between cars
	public double a_threshold = 0.2;
	public double a_bias = 0.05;
	public double autonomous_left_a_bias = 0.3;
	public double p = 0.7;
	public double default_cooldown = 3.0; //rough duration of a lane change
	public double a_max = 2.0; // m/s^2
	public double b_max = a_max * 4.0/3.0;
	public double b_safe = 4.0; //m/s
	public double v_max = 31.5; // m/s
	public double length = 4.0; // m
	
	public Vehicle front;
	public Vehicle back;
	public Vehicle leftFront;
	public Vehicle leftBack;
	public Vehicle rightFront;
	public Vehicle rightBack;
	
	public Pose pose;
	public Pose lastPose;
	
	public boolean used;
	
	public double lane_change_cooldown;
	
	public ArrayList<Integer> waypointsPassed = new ArrayList<Integer>();
	
	public Road road;
	
	public boolean autonomous;
	
	public Vehicle(double lane, Road road, boolean autonomous) {
		this.ID = total;
		total++;
		this.road = road;
		this.lastPose = new Pose(lane, road.lane_count);
		this.pose = lastPose;
		this.autonomous = autonomous;
		
		//human car variation
		if (!autonomous) {
			double agressiveness = Math.random();
			v_max = 28.0 + agressiveness*5.0;
			T = 1.2 + agressiveness*0.8;
			a_max = 1.75 + 0.5*agressiveness;
			b_max = a_max * 4.0/3.0;
			p = 0.5*(1-agressiveness);
		}
	}
	
	public Pose checkAcceleration(Vehicle front, Vehicle lane) {
		double v_max_eff = (autonomous && road.leftLaneFaster && ((lane == null && pose.lane == 0) || (lane != null && lane.pose.lane == 0))) ? v_max * 1.3 : v_max;
		Pose newPose = pose.copy();
		if (front == null) {
			newPose.gap_d = Optional.empty();
			newPose.a = a_max * (1-Math.pow((v_max == 0 ? 1 : pose.v/v_max_eff), 4));
		}
		else {
			double gap_a = 0;
			if (pose.lane == front.pose.lane) {
				gap_a = front.pose.s-pose.s-length;
			}
			else if (pose.lane > front.pose.lane && lane.pose.lane == pose.lane) {
				gap_a = front.pose.s_right.get()-pose.s-length;
			}
			else if (pose.lane > front.pose.lane && lane.pose.lane == front.pose.lane) {
				gap_a = front.pose.s-pose.s_left.get()-length;
			}
			else if (pose.lane < front.pose.lane && lane.pose.lane == pose.lane) {
				gap_a = front.pose.s_left.get()-pose.s-length;
			}
			else if (pose.lane < front.pose.lane && lane.pose.lane == front.pose.lane) {
				gap_a = front.pose.s-pose.s_right.get()-length;
			}
			double dv = pose.v-front.pose.v;
			
			
			newPose.gap_d = Optional.of(Math.max(0, pose.v*T + pose.v*dv / (2*Math.sqrt(a_max*b_max))) + s_0);
			newPose.a = a_max * (1-Math.pow((v_max == 0 ? 1 : pose.v/v_max_eff), 4) - Math.pow(newPose.gap_d.get()/gap_a,2));
			
			if (gap_a < s_0) {
				color = true;
//				System.out.println(ID + ": " + ((gap_a+length) < 0 ? "oof" : "gap too small"));
//				System.out.println("Gap: " + (gap_a+length));
//				System.out.println("Vel: " + pose.v);
//				System.out.println("Accel: " + a_max * (1-Math.pow((v_max == 0 ? 1 : pose.v/v_max), 4)));
//				System.out.println("Braking :" +  a_max * -Math.pow(newPose.gap_d.get()/gap_a,2));
			}
			else {
				color = false;
			}
		
		}
		return newPose;
	}
	
	public void accelerate() {
		pose = checkAcceleration(this.front, front);
	}
	
	public void update(double dt) {
		
		pose.v = lastPose.v + pose.a*dt;
		pose.s = lastPose.s + pose.v*dt + pose.a*dt*dt;

		//If calculated velocity < 0, don't move backwards
		if (pose.v < 0) {
//			System.out.println("Backwards?");
			pose.v = 0;
			pose.s = lastPose.s;
		}
		
		lane_change_cooldown -= dt;
		
		for (int i = 0; i < Points.waypoints.length; i++) {
			if (pose.t > Points.waypoints[i] && !waypointsPassed.contains(i)) {
				waypointsPassed.add(i);
				road.detector.addReading(i, pose.v);
			}
		}
		
	}
	
	public boolean changeRight(int lane_count) {
		
		for (Vehicle v: new Vehicle[]{this, rightBack, rightFront, front, back}) {
			if (v != null && v.lane_change_cooldown > 0) return false;
		}
		
		if (front == null || pose.lane > (lane_count-1) - Road.kEps) return false;
				
		if ((rightBack != null && this.pose.s_right.get()-rightBack.pose.s-length < s_0) || (rightFront != null && rightFront.pose.s-this.pose.s_right.get()-length < s_0)) return false;
		
		if ((rightBack != null && rightBack.checkAcceleration(this, rightBack).a < -b_safe) || (rightFront != null && checkAcceleration(rightFront, rightFront).a < -b_safe)) return false;
				
		if (checkAcceleration(rightFront, rightFront).a - checkAcceleration(front, front).a > p * ((rightBack == null) ? 0 : rightBack.checkAcceleration(rightFront, rightBack).a - rightBack.checkAcceleration(this, rightBack).a) + a_threshold - a_bias) {
			this.pose.lane += 1;
			lane_change_cooldown = default_cooldown;
			return true;
		}
		
		return false;
	}
	
	public boolean changeLeft(int lane_count) {
		for (Vehicle v: new Vehicle[]{this, leftBack, leftFront, front, back}) {
			if (v != null && v.lane_change_cooldown > 0) return false;
		}
		
		if (front == null || pose.lane < Road.kEps) return false;
		
		if (road.leftLaneAutonomousOnly && !this.autonomous && pose.lane < 1+Road.kEps) return false;
		
		if ((leftBack != null && this.pose.s_left.get()-leftBack.pose.s-length < s_0) || (leftFront != null && leftFront.pose.s-this.pose.s_left.get()-length < s_0)) return false;
		
		if ((leftBack != null && leftBack.checkAcceleration(this, leftBack).a < -b_safe) || (leftFront != null && checkAcceleration(leftFront, leftFront).a < -b_safe)) return false;

		if (checkAcceleration(leftFront, leftFront).a - checkAcceleration(front, front).a > p * ((leftBack == null) ? 0 : leftBack.checkAcceleration(leftFront, leftBack).a - leftBack.checkAcceleration(this, leftBack).a) + a_threshold + a_bias - (autonomous && (road.leftLaneFaster || road.leftLaneAutonomousOnly) ? autonomous_left_a_bias : 0)) {
//			System.out.println(checkAcceleration(leftFront, leftFront).a - checkAcceleration(front, front).a);
			this.pose.lane -= 1;
			lane_change_cooldown = default_cooldown;
			return true;
		}
		
		return false;
		
	}
	
	public double getS() {
		return pose.s;
	}
	public double[] getR() {
		return pose.r;
	}
	
	public void setWaypoints() {
		for (int i = 0; i < Points.waypoints.length; i++) {
			if (pose.t > Points.waypoints[i] && !waypointsPassed.contains(i)) {
				waypointsPassed.add(i);
			}
		}
	}
	
}
