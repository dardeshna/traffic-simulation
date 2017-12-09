package roadgenerator;

import java.util.ArrayList;

import util.InterpolatingDouble;
import util.InterpolatingTreeMap;
import static util.MatrixMath.*;

public class CubicSegmentPath {

	private ArrayList<CubicSegment> segments;
	private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ts_map;
	private double length;
	
	protected CubicSegmentPath(ControlPoint[] points) {
		
		segments = new ArrayList<CubicSegment>();
		
		for (int i = 0; i < points.length-1; i++) {
			ControlPoint p1 = points[i];
			ControlPoint p2 = points[i+1];
			
			double[][] p = new double[][] {
				p1.r(),
				p2.r(),
				p1.T_cartesian(),
				p2.T_cartesian(),
			};
			
			double[][] inverse_eq_coef = new double[][] {
				{1,0,0,0},
				{0,0,1,0},
				{-3,3,-2,-1},
				{2,-2,1,1}
			};
			
			double[][] result = transpose(multiply(inverse_eq_coef, p));
			
			segments.add(new CubicSegment(result));
			
		}

		int t_f = segments.size();
		
		ts_map = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
		
		double t = 0;
		double s = 0;
		double ds = 1e-1;
		
		while (t < t_f) {
			
			double ds_dt = segments.get((int)t).ds_dt(t-(int)t);
			double d2s_dt2 = segments.get((int)t).d2s_dt2(t-(int)t);
			
			double dt = (-ds_dt+Math.sqrt(ds_dt*ds_dt+2*d2s_dt2*ds))/(d2s_dt2);
			
			s += ds_dt*dt + d2s_dt2*dt*dt/2;
			
			ts_map.put(new InterpolatingDouble(s), new InterpolatingDouble(t));
			
			t += dt;
		}
		length = s;
	}
	
	
	
	public double[] r(double s) {
		InterpolatingDouble s_1 = new InterpolatingDouble(s);
		double t;
		if (ts_map.higherKey(s_1) == null) {
			t = ts_map.lastEntry().getValue().value;
		}
		else if (ts_map.lowerKey(s_1) == null) {
			t = ts_map.firstEntry().getValue().value;
		}
		else {
			t = ts_map.getInterpolated(s_1).value;
		}
		t = Math.min(Math.max(0.0, t), segments.size());
		return segments.get((int)t).r(t-(int)t);
	}
	
	public double getPathLength() {
		return length;
	}
	
	public int numPathSegments() {
		return segments.size();
	}
	
}
