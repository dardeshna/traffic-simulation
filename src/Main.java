import roadgenerator.ControlPoint;
import roadgenerator.Road;
import vehicle.Vehicle;

public class Main {

	public static void main(String[] args) {
		double[][] points = new double[][] {
			{0,0,0,400},
			{400,175,Math.PI/4, 400},
			{800,350,0, 400},
		};
		
		ControlPoint[] controlPoints = ControlPoint.fromArray(points);
		
		Road road = new Road(controlPoints);
		
		double dt = road.dt;
		double warp = 3;
		
		do {
			road.update();
			long now = System.nanoTime();
			long wait = now+(long)(1e9*dt)/((long)(Math.pow(2, warp-1)));
			while (now < wait) {
				now = System.nanoTime();
			}
		} while (true);
		
		
	}
	
}
