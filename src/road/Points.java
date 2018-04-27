package road;

public class Points {

	public static double[][] points;
	public static double[] waypoints;
	
	static {
		setHighwayPath();
	}
	
	public static void setTestPath() {
		points = new double[][] {
			{0,0,0,400},
			{400,175,Math.PI/4, 400},
			{800,350,0, 400},
		};
		waypoints = new double[] {0.5, 1.5};
	}
	
	public static void setHighwayPath() {
		points = new double[][] {
			{0, 0, 0.8352102961, 500},
			{803, 887, 0.369166657, 750},
			{1081, 995, 0.3644596102, 200},
			{1366, 1104, 0.2556989195, 200},
			{2017, 1274, 0.6251446396, 750},
			{3258, 2169, 0.5874172606, 500}
		};
		waypoints = new double[] {0.5, 2.5, 4.5};
	}
	
}
