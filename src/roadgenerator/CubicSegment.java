package roadgenerator;

public class CubicSegment {

	/*
	 * Conventions:
	 * r -> 2D position
	 * t -> parametric position (each segment has length 1)
	 * s -> arc length position (meters)
	 */
	
	private double[][] coef;
	private double offset;  //For segments that represent adjacent lanes
	
	public CubicSegment(double[][] coef) {
		
	}
	public CubicSegment(double[][] coef, double offset) {
		this.coef = coef.clone();
		this.offset = offset;
	}
	
	public double[][] coef() {
		return coef.clone();
	}
	
	public double[] r(double t) {
		t = Math.min(Math.max(0.0, t), 1.0);
		return new double[]{x(t), y(t)};
	}
	
	public double x(double t) {
		double x = coef[0][0]+coef[0][1]*t+coef[0][2]*t*t+coef[0][3]*t*t*t;
		double dx = -(coef[1][1]+2*coef[1][2]*t+3*coef[1][3]*t*t)/v(t);
		return x+dx*offset;
	}
	public double y(double t) {
		double y = coef[1][0]+coef[1][1]*t+coef[1][2]*t*t+coef[1][3]*t*t*t;
		double dy = (coef[0][1]+2*coef[0][2]*t+3*coef[0][3]*t*t)/v(t);
		return y+dy*offset;
	}
	
	private double v(double t) {
		t = Math.min(Math.max(0.0, t), 1.0);
		double dx_dt = coef[0][1]+2*coef[0][2]*t+3*coef[0][3]*t*t;
		double dy_dt = coef[1][1]+2*coef[1][2]*t+3*coef[1][3]*t*t;
		return Math.sqrt(dx_dt*dx_dt+dy_dt*dy_dt);
	}
	
	public double ds_dt(double t) {
		t = Math.min(Math.max(0.0, t), 1.0);
		double dx_dt = (x(t+1e-6)-x(t))/1e-6;
		double dy_dt = (y(t+1e-6)-y(t))/1e-6;
		return Math.sqrt(dx_dt*dx_dt+dy_dt*dy_dt);
	}
	
	public double d2s_dt2(double t) {
		return (ds_dt(t+1e-6)-ds_dt(t))/1e-6;
	}
	
	/*
	public double d2s_dt2(double t) {
		double b = coef[0][1];
		double c = coef[0][2];
		double d = coef[0][3];
		double f = coef[1][1];
		double g = coef[1][2];
		double h = coef[1][3];
		return (2*(2*c+6*d*t)*(b+2*c*t*3*d*t*t)+2*(2*g+6*h*t)*(f+2*g*t+3*h*t*t))/(2*ds_dt(t));
	}
	*/
	
}
