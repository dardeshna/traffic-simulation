package road;

import java.io.PrintWriter;
import java.util.ArrayList;

public class Detector {
	
	public static final double printdt = 1.0;
	public static final boolean print = true;
	
	public ArrayList<ArrayList<Double>>[] readings;
	public ArrayList<Double>[] latestReadings;
	
	public PrintWriter writer;
	
	public double counter = 0.0;
	
	@SuppressWarnings("unchecked")
	public Detector(String filename) {
		readings = (ArrayList<ArrayList<Double>>[]) new ArrayList[Points.waypoints.length];
		for (int i = 0; i < readings.length; i++) {
			readings[i] = new ArrayList<ArrayList<Double>>();
		}
		latestReadings = (ArrayList<Double>[]) new ArrayList[Points.waypoints.length];
		for (int i = 0; i < latestReadings.length; i++) {
			latestReadings[i] = new ArrayList<Double>();
		}
		try {
			if (print) writer = new PrintWriter(filename, "UTF-8");
		}
		catch (Exception e) {
			//nothing
		}

	}
	
	public void addReading(int waypoint, double reading) {
		latestReadings[waypoint].add(reading);
	}
	
	@SuppressWarnings("unused")
	public void update(double dt) {
		counter += dt;
		for (int i = 0; i < readings.length; i++) {
			readings[i].add(latestReadings[i]);
			if (readings[i].size() > 30 / dt) readings[i].remove(0);
			if (counter >= Detector.printdt && Detector.print) {
				int numReadings = 0;
				int sumReadings = 0;
				for (int j = 0; j < readings[i].size(); j++) {
					for (int k = 0; k < readings[i].get(j).size(); k++) {
						numReadings++;
						sumReadings+=readings[i].get(j).get(k);
					}
				}
				double avgReading = (numReadings==0) ? 0 : (double)sumReadings/numReadings;
				double hourlyFlow = numReadings * 120;
				writer.print(avgReading + "," + hourlyFlow + ",");
			}
		}
		if (counter >= Detector.printdt && Detector.print) {
			writer.println();
			writer.flush();
			counter = 0;
		}
		for (int i = 0; i < latestReadings.length; i++) {
			latestReadings[i] = new ArrayList<Double>();
		}
	}

}
