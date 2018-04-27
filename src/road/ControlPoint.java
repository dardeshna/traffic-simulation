package road;

public class ControlPoint {

	private double x;
	private double y;
	private double theta;
	private double r;
	
	public ControlPoint(double x, double y, double theta, double r) {
		this.x = x;
		this.y = y;
		if (r < 0) {
			theta = theta+Math.PI;
			r = -r;
		}
		this.theta = theta;
		this.r = r;
	}
	
	public double[] r() {
		return new double[] {x, y};
	}
	
	public double[] T() {
		return new double[] {r, theta};		
	}
	
	public double[] T_cartesian() {
		return new double[] {r*Math.cos(theta), r*Math.sin(theta)};
	}
	
	public static ControlPoint[] fromArray(double[][] points) {
		
		ControlPoint[] created_points = new ControlPoint[points.length];
		
		for (int i = 0; i < points.length; i++) {
			created_points[i] = new ControlPoint(points[i][0], points[i][1], points[i][2], points[i][3]);
		}
		
		return created_points;
	}
	
}
