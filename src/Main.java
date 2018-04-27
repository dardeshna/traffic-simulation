import java.awt.event.WindowEvent;

import road.Points;
import road.Road;

public class Main {
	
	public static final boolean singleTest = true;
	
	public static final boolean enableDelay = true;
	public static final double warp = 3;
	
	public static final boolean enableGraphics = true;
	
	public static final double time = 20*60;
	
	public static final int trials = 1;

	public static void main(String[] args) {
		
		Road.enableGraphics = enableGraphics;
		
		if (singleTest) {
			
//			Points.setTestPath();
			
			Road road = new Road(Points.points);
						
//			road.leftLaneAutonomousOnly = true;
//			road.leftLaneFaster = true;
//			road.autonomousPercentage = 1;

			road.initDetector(null);
			
			road.populateVehicles();
			double dt = road.dt;
			double t = 0;

			do {
				road.update();
				if (enableDelay) {
					long now = System.nanoTime();
					long wait = now+(long)((1e9*dt)/((Math.pow(2, warp-1))));
					while (now < wait) {
						now = System.nanoTime();
					}
				}
				t+=dt;
			} while (t < time || time < 0);
			
		}
		else {
			
			double autonomousPercentage = 0.0;
			boolean leftLaneAutonomousOnly = false;
			boolean leftLaneFaster = false;
			boolean highFlow = false;
			int count = 0;
			while (autonomousPercentage <= 1.0) {
				
				Road road = new Road(Points.points);
				road.inflow = (highFlow ? 5000 : 2500);
				road.autonomousPercentage = autonomousPercentage;
				road.leftLaneAutonomousOnly = leftLaneAutonomousOnly;
				road.leftLaneFaster = leftLaneFaster;
				road.initDetector(((Integer)count).toString());
				
				road.populateVehicles();
				
				double dt = road.dt;
				double t = 0;
	
				do {
					road.update();
					if (enableDelay) {
						long now = System.nanoTime();
						long wait = now+(long)((1e9*dt)/((Math.pow(2, warp-1))));
						while (now < wait) {
							now = System.nanoTime();
						}
					}
					t+=dt;
				} while (t < time || time < 0);
				
				road.dispatchEvent(new WindowEvent(road, WindowEvent.WINDOW_CLOSING));
				
				System.out.println("Complete [trial " + count + "]: " + (highFlow ? "highFlow" : "lowFlow") + ", " + autonomousPercentage + ", " + (leftLaneAutonomousOnly ? "leftLaneAutonomous" : "leftLaneMixed") + ", " + (leftLaneFaster ? "leftLaneFast" : "leftLaneSlow"));

				
				count++;
				if (count >= trials) {
					if (!highFlow) {
						highFlow = true;
					}
					else {
						highFlow = false;
						autonomousPercentage += 0.2;
						if (autonomousPercentage > 1.0) {
							if (!leftLaneAutonomousOnly) {
								leftLaneAutonomousOnly = true;
								autonomousPercentage = 0.0;
							}
							else if (!leftLaneFaster) {
								leftLaneFaster = true;
								autonomousPercentage = 0.0;
							}
							
						}
					}
					count = 0;
				}
				
			}
			
		}
		
	System.out.println("Simulation Finished");
	
	}
	
}
