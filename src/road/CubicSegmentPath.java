package road;

import java.util.ArrayList;

import util.InterpolatingDouble;
import util.InterpolatingTreeMap;
import static util.MatrixMath.*;

public class CubicSegmentPath {
	
	/*
	 * Conventions:
	 * r -> 2D position
	 * t -> parametric position (each segment has length 1)
	 * s -> arc length position (meters)
	 */

	ArrayList<CubicSegment> segments;
	InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ts_map;
	InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> st_map;

	double length;
	double offset;
	
	CubicSegmentPath() {
		
	}
	
	public CubicSegmentPath(ControlPoint[] points) {
		this(points, 0);
	}
	
	public CubicSegmentPath(ControlPoint[] points, double offset) {
		
		this.offset = offset;
		
		segments = new ArrayList<CubicSegment>();
		
		//Calculate cubic splines between control points
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
			
//			for (double[] j: result) {
//				for (double k: j) {
//					System.out.print(k + " ");
//				}
//				System.out.println();
//			}
			
			segments.add(new CubicSegment(result, this.offset));
			
		}
		//Arc length parameterize the entire path

		int t_f = segments.size();
		
		ts_map = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
		st_map = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

		
		double t = 0;
		double s = 0;
		double ds = 1e-1;
		
		while (t < t_f) {
			
			double ds_dt = segments.get((int)t).ds_dt(t-(int)t);
			
			
						
			double d2s_dt2 = segments.get((int)t).d2s_dt2(t-(int)t);
			
			double dt = (-ds_dt+Math.sqrt(Math.max(ds_dt*ds_dt+2*d2s_dt2*ds, 0)))/(d2s_dt2);
			
			if(new Double(dt).equals(Double.NaN)) {
				t += dt;
				continue;
			}
			
			s += ds_dt*dt + d2s_dt2*dt*dt/2;
			
			ts_map.put(new InterpolatingDouble(s), new InterpolatingDouble(t));
			st_map.put(new InterpolatingDouble(t), new InterpolatingDouble(s));
			
			t += dt;
			
		}
		length = s;
		
		
		///System.out.println(s);
		
	}
	
	//Return 2D position given arc length position
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
		t = Math.min(Math.max(0.0, t), segments.size()-(1e-12));
		return segments.get((int)t).r(t-(int)t);
	}
	
	//Return parametric position given arc length position
	public double t(double s) {
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
		t = Math.min(Math.max(0.0, t), segments.size()-(1e-12));
		return t;
	}
	
	public double s(double t) {
		InterpolatingDouble t_1 = new InterpolatingDouble(t);
		double s;
		if (st_map.higherKey(t_1) == null) {
			s = st_map.lastEntry().getValue().value;
		}
		else if (st_map.lowerKey(t_1) == null) {
			s = st_map.firstEntry().getValue().value;
		}
		else {
			s = st_map.getInterpolated(t_1).value;
		}
		s = Math.min(Math.max(0.0, s), length-(1e-12));
		return s;
	}
	
	public double getPathLength() {
		return length;
	}
	
	public int numPathSegments() {
		return segments.size();
	}
	
}
