package archive;
import java.util.ArrayList;

public class RoadGenerator {

	public static void main(String[] args) {
		double[][] controlPoints = new double[][] {
			{0,0,0,50},
			{100,100,Math.PI,50},
			{100,200,3*Math.PI/4,50},
			{200,300,0,50}
		};
		
		double[][][] coef = computePolynomials(controlPoints);
		
		for (int i = 0; i < coef.length; i++) {
			for (int j = 0; j < coef[i].length; j++) {
				System.out.print(coef[i][j][0] + " + " + coef[i][j][1] + "t + " + coef[i][j][2] + "t^2 + " + coef[i][j][3] + "t^3 ");
				System.out.print("| ");
			}
			System.out.println();
		}
		
		int t_f = coef.length;
		
		@SuppressWarnings("unchecked")
		ArrayList<Double>[] ts_map = (ArrayList<Double>[]) new ArrayList[] {new ArrayList<Double>(), new ArrayList<Double>()};
		
		double t = 0;
		double s = 0;
		double ds = 1e-1;
		
		while (t<t_f) {
			
			double ds_dt = ds_dt(coef[(int)t], t-(int)t);
			double d2s_dt2 = (ds_dt(coef[(int)t], t-(int)t+(1e-7))-ds_dt)/(1e-7);
			
			double dt = (-ds_dt+Math.sqrt(ds_dt*ds_dt+2*d2s_dt2*ds))/(d2s_dt2);
			
			s += ds_dt*dt + d2s_dt2*dt*dt/2;
			
			ts_map[0].add(t);
			ts_map[1].add(s);
			
			t+=dt;
			
		}
		
		System.out.println(s);
		for(int i = 0; i < ts_map[0].size(); i++) {
			System.out.println(ts_map[1].get(i) + ", " + ts_map[0].get(i));
		}
		
	}
	
	public static double ds_dt(double[][] poly_coef, double t) {
		double dx_dt = poly_coef[0][1]+2*poly_coef[0][2]*t+3*poly_coef[0][3]*t*t;
		double dy_dt = poly_coef[1][1]+2*poly_coef[1][2]*t+3*poly_coef[1][3]*t*t;
		return Math.sqrt(dx_dt*dx_dt+dy_dt*dy_dt);
	}
	
	public static double[][][] computePolynomials(double[][] controlPoints) {
		
		double[][][] poly_coef = new double[controlPoints.length-1][2][4];
		
		
		for (int i = 0; i < controlPoints.length-1; i++) {
			double[] p1 = controlPoints[i];
			double[] p2 = controlPoints[i+1];
			
			double[][] p = new double[][] {
				{p1[0],p1[1]},
				{p2[0],p2[1]},
				{Math.cos(p1[2])*p1[3], Math.sin(p1[2])*p1[3]},
				{Math.cos(p2[2])*p2[3], Math.sin(p2[2])*p2[3]},
			};
			
			double[][] inverse_eq_coef = new double[][] {
				{1,0,0,0},
				{0,0,1,0},
				{-3,3,-2,-1},
				{2,-2,1,1}
			};
			
			double[][] result = transpose(multiply(inverse_eq_coef, p));
			
			poly_coef[i] = result;
			
			}
		
		return poly_coef;
	}
	
	public static double[][] multiply(double[][] m1, double[][] m2) {
        int m1ColLength = m1[0].length; // m1 columns length
        int m2RowLength = m2.length;    // m2 rows length
        if(m1ColLength != m2RowLength) return null; // matrix multiplication is not possible
        int mRRowLength = m1.length;    // m result rows length
        int mRColLength = m2[0].length; // m result columns length
        double[][] mResult = new double[mRRowLength][mRColLength];
        for(int i = 0; i < mRRowLength; i++) {         // rows from m1
            for(int j = 0; j < mRColLength; j++) {     // columns from m2
                for(int k = 0; k < m1ColLength; k++) { // columns from m1
                    mResult[i][j] += m1[i][k] * m2[k][j];
                }
            }
        }
        return mResult;
    }

	public static double[][] transpose(double[][] m) {
		double[][] temp = new double[m[0].length][m.length];
		for (int i = 0; i < m.length; i++)
			for (int j = 0; j < m[0].length; j++)
				temp[j][i] = m[i][j];
		return temp;
	}
	
}
