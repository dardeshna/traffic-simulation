package roadgenerator;

public class CubicSegment {

	private double[][] coef;
	
	public CubicSegment(double[][] coef) {
		this.coef = coef.clone();
	}
	
	public double[][] coef() {
		return coef.clone();
	}
	
	public double[] r(double t) {
		t = Math.min(Math.max(0.0, t), 1.0);
		double x = coef[0][0]+coef[0][1]*t+coef[0][2]*t*t+coef[0][3]*t*t*t;
		double y = coef[1][0]+coef[1][1]*t+coef[1][2]*t*t+coef[1][3]*t*t*t;
		return new double[] {x,y};
	}
	
	public double ds_dt(double t) {
		t = Math.min(Math.max(0.0, t), 1.0);
		double dx_dt = coef[0][1]+2*coef[0][2]*t+3*coef[0][3]*t*t;
		double dy_dt = coef[1][1]+2*coef[1][2]*t+3*coef[1][3]*t*t;
		return Math.sqrt(dx_dt*dx_dt+dy_dt*dy_dt);
	}
	
	public double d2s_dt2(double t) {
		t = Math.min(Math.max(0.0, t), 1.0);
		return (ds_dt(t+(1e-7))-ds_dt(t))/(1e-7);
	}
}
