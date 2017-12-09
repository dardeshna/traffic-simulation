import roadgenerator.ControlPoint;
import roadgenerator.Road;

public class Main {

	public static void main(String[] args) {
		double[][] points = new double[][] {
			{0,0,0,50},
			{100,100,Math.PI,50},
			{100,200,3*Math.PI/4,50},
			{200,300,0,50}
		};
		
		ControlPoint[] controlPoints = ControlPoint.fromArray(points);
		
		Road road = new Road(controlPoints);
		
		double u = 0;
		
		while (u < road.getPathLength()) {
			double[] r = road.r(u);
			System.out.println(r[0] + ", " + r[1]);
			u+=1;
		}
		
	}
	
}
