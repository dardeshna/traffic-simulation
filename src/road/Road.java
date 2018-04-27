package road;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.Optional;

import javax.swing.JFrame;
import javax.swing.JPanel;

import vehicle.Vehicle;


public class Road extends JFrame {
	
	public static boolean enableGraphics = true;
	public static final double kEps = 1e-5;

	private static final long serialVersionUID = 1L;
	public static final int WIDTH = 800;  //Width of frame
	public static final int HEIGHT = WIDTH/2;  //Height of frame
	public static final int CONVERSION_FACTOR = WIDTH/8;

	public ArrayList<Vehicle> vehicles = new ArrayList<Vehicle>();
	public int lane_count = 4;
	public double lane_spacing = 3.7;  //meters
	public CubicSegmentPath[] lanes = new CubicSegmentPath[lane_count];
	public Detector detector;
	
	public boolean leftLaneAutonomousOnly = true;
	public boolean leftLaneFaster = true;
	
	public double t;
	public int counter;
	public double dt = 0.05;
	public double inflow = 6000; //veh/hr

	public double starting_speed = 1.0;
	public double speed_limit = 30.5;
	public double density = inflow/(speed_limit * 3.6); //veh/km
	public double laneAddingThreshold = 10.0; //meters
	
	public double autonomousPercentage = 0;
	
	public Road(double[][] points_array) {
		ControlPoint[] points = ControlPoint.fromArray(points_array);
		for (int i = 0; i < lanes.length; i++) {
			lanes[i] = new CubicSegmentPath(points, i*lane_spacing);
		}
		
		if (enableGraphics) {
		
			JPanel panel = new JPanel() {
				
				//Window zoom and pan controls
				
				private int c_x;
				private int c_y;
				private int last_x;
				private int last_y;
				private boolean dragging;
				private int zoom = -1;
				
				{
				addMouseMotionListener(new MouseMotionListener() {
	
					@Override
					public void mouseDragged(MouseEvent e) {
						if (!dragging) {
							dragging = true;
						}
						else {
							c_x += e.getX()-last_x;
							c_y += e.getY()-last_y;
						}
						
						last_x = e.getX();
						last_y = e.getY();
					}
	
					@Override
					public void mouseMoved(MouseEvent e) {
						dragging = false;
					}
					
				});
				
				addKeyListener(new KeyListener() {
	
					@Override
					public void keyTyped(KeyEvent e) {
						if (e.getKeyChar() == '+' || e.getKeyChar() == '=') {
							zoom += 1;
						}
						else if (e.getKeyChar() == '-' ||e.getKeyChar() == '_') {
							zoom -= 1;
						}
						
					}
	
					@Override
					public void keyPressed(KeyEvent e) { }
	
					@Override
					public void keyReleased(KeyEvent e) { }
			        
				});
				}
				
				private static final long serialVersionUID = 1L;
				
				@Override
				public void paintComponent(Graphics g) {
					paintComponent((Graphics2D) g);
				}
				
				/**
				 * 
				 * Paints the window, road and vehicles
				 * @param graphics
				 */
				public void paintComponent(Graphics2D graphics) {
					this.setFocusable(true);
					this.requestFocusInWindow();
					
		            graphics.translate(c_x, c_y);
		            graphics.scale(Math.pow(2, zoom-1), Math.pow(2, zoom-1));
		            
					graphics.setColor(Color.GRAY);
					
					double ds = 1;
					
					graphics.setStroke(new BasicStroke(2));
					
					//Paint lanes
					for (CubicSegmentPath p: lanes) {
						if (p != null) {
							double s = 0;
							while (s < p.getPathLength()) {
								double[] r1 = p.r(s);
								double[] r2 = p.r(s+ds);
								graphics.drawLine((int)(4*r1[0]), (int)(4*r1[1]), (int)(4*r2[0]), (int)(4*r2[1]));
								s+=ds;
							}
						}
					}
					
					graphics.setStroke(new BasicStroke(4));
					
					//Paint vehicles
					for (int i = 0; i < vehicles.size(); i++) {
						double r[] = vehicles.get(i).getR();

						graphics.setColor(Color.BLUE);
						if (vehicles.get(i).autonomous) graphics.setColor(Color.RED);
						if (vehicles.get(i).color) graphics.setColor(Color.GREEN);

						graphics.drawOval((int)(4*(r[0]-2)), (int)(4*(r[1]-2)), 16, 16);
					}
					
				}
				
				@Override
				public Dimension getPreferredSize() {
					return new Dimension(Road.WIDTH, Road.HEIGHT);
				}
				
			};
			
			setVisible(true);
			
			setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			
			Container c = getContentPane();
			c.setLayout(new BorderLayout());
			
			setBackground(Color.WHITE);
			c.add(panel, BorderLayout.CENTER);
			
			pack();
		}
		
	}
	
	public void initDetector(String uniqueString) {
		
		String name = "output-" + (int) inflow + "-" + (int) (autonomousPercentage * 100) + "-" + (leftLaneAutonomousOnly ? "leftLaneAutonomous" : "leftLaneMixed") + "-" + (leftLaneFaster ? "leftLaneFast" : "leftLaneSlow");
		
		if (uniqueString != null) name += "-" + uniqueString;
		
		detector = new Detector(name + ".csv");
	}
	
	/**
	 * Updates the vehicle pose and surrounding vehicles
	 * @param dt timestep
	 */
	public void updateVehicles(double dt) {
		ArrayList<Vehicle> finished = new ArrayList<Vehicle>();
		for (Vehicle v: vehicles) {
			v.accelerate();
		}
		
		//Update vehicle pose
		for (Vehicle v: vehicles) {
			if (v.pose.a > v.a_max) {
				System.out.println("ruh roh");
			}
			v.update(dt);
			v.pose.r = lanes[(int)v.pose.lane].r(v.getS());
			v.pose.t = lanes[(int)v.pose.lane].t(v.getS());
			
			v.pose.s_left = ((int)v.pose.lane-1 >= 0) ? Optional.of(lanes[(int)v.pose.lane-1].s(v.pose.t)) : Optional.empty();
			v.pose.s_right = ((int)v.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)v.pose.lane+1].s(v.pose.t)) : Optional.empty();
			
			if (v.getS() >= lanes[(int)v.pose.lane].getPathLength()) {
				finished.add(v);
				v.used = true;
			}
			
		}
		
		//Remove finished vehicles
		for (Vehicle v: finished) {
			vehicles.remove(v);
		}
		
		updateSurrounding();
		
//		ArrayList<Vehicle> test = new ArrayList<Vehicle>();
//		
//		for (Vehicle v: vehicles) {
//			test.add(v);
//			test.add(v.front);
//			test.add(v.leftFront);
//			test.add(v.rightFront);
//			test.add(v.back);
//			test.add(v.leftBack);
//			test.add(v.rightBack);
//		}
			
		for (Vehicle v: vehicles) {
			if (v.changeRight(lane_count) == true) {
//				System.out.println("to right: " + v.ID);
				v.pose.s = v.pose.s_right.get();
				v.pose.t = lanes[(int)v.pose.lane].t(v.getS());
				v.pose.s_left = ((int)v.pose.lane-1 >= 0) ? Optional.of(lanes[(int)v.pose.lane-1].s(v.pose.t)) : Optional.empty();
				v.pose.s_right = ((int)v.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)v.pose.lane+1].s(v.pose.t)) : Optional.empty();

				updateSurrounding();
				
			}
		}
		
		for (Vehicle v: vehicles) {
			if (v.changeLeft(lane_count) == true) {
//				System.out.println("to left: " + v.ID);
				v.pose.s = v.pose.s_left.get();
				v.pose.t = lanes[(int)v.pose.lane].t(v.getS());
				v.pose.s_left = ((int)v.pose.lane-1 >= 0) ? Optional.of(lanes[(int)v.pose.lane-1].s(v.pose.t)) : Optional.empty();
				v.pose.s_right = ((int)v.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)v.pose.lane+1].s(v.pose.t)) : Optional.empty();

				updateSurrounding();
				
			}
		}
		
		
		updateSurrounding();
		
//		System.out.println("--------------------------");
//		for (Vehicle v: vehicles) {
//			ArrayList<Vehicle> all = new ArrayList<Vehicle>(Arrays.asList(new Vehicle[]{v, v.front, v.back, v.rightFront, v.rightBack, v.leftFront, v.leftBack}));
//			System.out.print(v.pose.lane + ": ");
//			for (Vehicle w: all) {
//				System.out.print((w==null ? null : w.ID) + ", ");
//			}
//			System.out.println();
//		}
		
//		ArrayList<Vehicle> test2 = new ArrayList<Vehicle>();
//		
//		for (Vehicle v: vehicles) {
//			test2.add(v);
//			test2.add(v.front);
//			test2.add(v.leftFront);
//			test2.add(v.rightFront);
//			test2.add(v.back);
//			test2.add(v.leftBack);
//			test2.add(v.rightBack);
//		}
//		
//		for (int i=0; i < test.size(); i++) {
//			if (i%7==0) System.out.print(test.get(i).ID + ": ");
//			if (test.get(i) != test2.get(i)) {
//				System.out.print(i%7 + "[" + (test.get(i)==null ? null : test.get(i).ID) + ","+ (test2.get(i)==null ? null : test2.get(i).ID) +"], ");
//			}
//			if (i%7==6) System.out.println();
//		}
//		
		if (counter > 3600 / inflow / dt) {
			addVehicles();
			counter = 0;
		}
		else {
			counter++;
		}
		
		for (Vehicle v: vehicles) {
			v.lastPose = v.pose.copy();
		}
		
	}
	
	@SuppressWarnings("unused")
	private void updateSurroundingNew() {
		for (Vehicle v: vehicles) {
			ArrayList<Vehicle> currentSet = new ArrayList<Vehicle>(Arrays.asList(new Vehicle[]{v.front, v.back, v.rightFront, v.rightBack, v.leftFront, v.leftBack}));
			
			boolean allExist = true;
			for (Vehicle w: currentSet) {
				if (w == null) allExist = false;
			}
						
			if (!allExist) {
				for (Vehicle w: vehicles) {
					
					if (v==w) {
						continue;
					}
					else if (Math.abs(w.pose.lane - v.pose.lane) <= 1-kEps) {
						if (w.pose.t < v.pose.t && (v.back == null || w.pose.t > v.back.pose.t)) {
							v.back = w;
						}
						if (w.pose.t > v.pose.t && (v.front == null || w.pose.t < v.front.pose.t)) {
							v.front = w;
						}
					}
					
					//right
					else if ((w.pose.lane - v.pose.lane) > 1-kEps && (w.pose.lane - v.pose.lane) <= 2-kEps) {
						if (w.pose.t < v.pose.t && (v.rightBack == null || w.pose.t > v.rightBack.pose.t)) {
							v.rightBack = w;
						}
						if (w.pose.t > v.pose.t && (v.rightFront == null || w.pose.t < v.rightFront.pose.t)) {
							v.rightFront = w;
						}
					}
					//left
					else if ((w.pose.lane - v.pose.lane) < -1+kEps && (w.pose.lane - v.pose.lane) >= -2+kEps) {
						if (w.pose.t < v.pose.t && (v.leftBack == null || w.pose.t > v.leftBack.pose.t)) {
							v.leftBack = w;
						}
						if (w.pose.t > v.pose.t && (v.leftFront == null || w.pose.t < v.leftFront.pose.t)) {
							v.leftFront = w;
						}
					}
				}
			}
			
			else {
			
				ArrayList<Vehicle> newSet = new ArrayList<Vehicle>();
				
				@SuppressWarnings("unchecked")
				HashSet<Vehicle> surrounding = new HashSet<Vehicle>((ArrayList<Vehicle>) currentSet.clone());
				surrounding.add(v);
				surrounding.remove(null);
				
				boolean equalSets = false;
				
				while (!equalSets) {
					currentSet = newSet;
					for (Vehicle w: surrounding) {
					
						if (v==w) {
							continue;
						}
						else if (Math.abs(w.pose.lane - v.pose.lane) <= 1-kEps) {
							if (w.pose.t < v.pose.t && (v.back == null || w.pose.t > v.back.pose.t)) {
								v.back = w;
							}
							if (w.pose.t > v.pose.t && (v.front == null || w.pose.t < v.front.pose.t)) {
								v.front = w;
							}
						}
						
						//right
						else if ((w.pose.lane - v.pose.lane) > 1-kEps && (w.pose.lane - v.pose.lane) <= 2-kEps) {
							if (w.pose.t < v.pose.t && (v.rightBack == null || w.pose.t > v.rightBack.pose.t)) {
								v.rightBack = w;
							}
							if (w.pose.t > v.pose.t && (v.rightFront == null || w.pose.t < v.rightFront.pose.t)) {
								v.rightFront = w;
							}
						}
						//left
						else if ((w.pose.lane - v.pose.lane) < -1+kEps && (w.pose.lane - v.pose.lane) >= -2+kEps) {
							if (w.pose.t < v.pose.t && (v.leftBack == null || w.pose.t > v.leftBack.pose.t)) {
								v.leftBack = w;
							}
							if (w.pose.t > v.pose.t && (v.leftFront == null || w.pose.t < v.leftFront.pose.t)) {
								v.leftFront = w;
							}
						}
						
					}
					newSet = new ArrayList<Vehicle>(Arrays.asList(new Vehicle[]{v.front, v.back, v.rightFront, v.rightBack, v.leftFront, v.leftBack}));
					HashSet<Vehicle> oldSurrounding = (HashSet<Vehicle>) surrounding.clone();
					for (Vehicle w: oldSurrounding) {
						surrounding.addAll(Arrays.asList(new Vehicle[]{w.front, w.back, w.rightFront, w.rightBack, w.leftFront, w.leftBack}));
					}
					surrounding.remove(null);
					
					equalSets = equalSets(currentSet, newSet);
				}
			}
		
		}
		
		for (Vehicle v: vehicles) {
			if (v.front != null && v.front.pose.s < v.pose.t) {
				System.out.println("front issue");
			}
			if (v.leftFront != null && v.leftFront.pose.s_right.get() < v.pose.s) {
				System.out.println("front left issue");
			}
			if (v.rightFront != null && v.rightFront.pose.s_left.get() < v.pose.s) {
				System.out.println("front right issue");
			}
			
			if (v.back != null && v.back.pose.t > v.pose.s) {
				System.out.println("back issue");
			}
			if (v.leftBack != null && v.leftBack.pose.s_right.get() > v.pose.s) {
				System.out.println("back left issue");
			}
			if (v.rightBack != null && v.rightBack.pose.s_left.get() > v.pose.s) {
				System.out.println("back right issue");
			}
		}

	}
	
	private void updateSurrounding() {
		//Update surrounding vehicles for each vehicle
		for (Vehicle v: vehicles) {
			v.front = null;
			v.back = null;
			v.rightFront = null;
			v.rightBack = null;
			v.leftFront = null;
			v.leftBack = null;
			for (Vehicle w: vehicles) {

				if (v==w) {
					continue;
				}
				else if (Math.abs(w.pose.lane - v.pose.lane) <= 1-kEps) {
					if (w.pose.t < v.pose.t && (v.back == null || w.pose.t > v.back.pose.t)) {
						v.back = w;
					}
					if (w.pose.t > v.pose.t && (v.front == null || w.pose.t < v.front.pose.t)) {
						v.front = w;
					}
				}
				
				//right
				else if ((w.pose.lane - v.pose.lane) > 1-kEps && (w.pose.lane - v.pose.lane) <= 2-kEps) {
					if (w.pose.t < v.pose.t && (v.rightBack == null || w.pose.t > v.rightBack.pose.t)) {
						v.rightBack = w;
					}
					if (w.pose.t > v.pose.t && (v.rightFront == null || w.pose.t < v.rightFront.pose.t)) {
						v.rightFront = w;
					}
				}
				//left
				else if ((w.pose.lane - v.pose.lane) < -1+kEps && (w.pose.lane - v.pose.lane) >= -2+kEps) {
					if (w.pose.t < v.pose.t && (v.leftBack == null || w.pose.t > v.leftBack.pose.t)) {
						v.leftBack = w;
					}
					if (w.pose.t > v.pose.t && (v.leftFront == null || w.pose.t < v.leftFront.pose.t)) {
						v.leftFront = w;
					}
				}
				
			}
		}		
	}
		
	private static boolean equalSets(ArrayList<Vehicle> currentSet, ArrayList<Vehicle> newSet) {
		if (currentSet.size() != newSet.size()) {

			return false;
		}
		
		for (int i = 0; i < currentSet.size(); i++) {
			if (currentSet.get(i) != newSet.get(i)) return false;
		}
		
		return true;
	}
	
	public void updateGraphics() {
		this.repaint();
	}
	
	/**
	 * Main update function for the simulation
	 */
	public void update() {
		updateVehicles(dt);
		detector.update(dt);
		updateGraphics();
		t += dt;
	}

	/** 
	 * Adds new vehicles to the road
	 */
	private void addVehicles() {
		
		boolean isAutonomous = Math.random() < autonomousPercentage;
		
		Vehicle[] closest = new Vehicle[lane_count];
		
		//Obtains the last vehicle in each lane
		for (Vehicle v: vehicles) {
			for (int i = 0; i < lane_count; i++) {
				if (v.pose.lane == i) {
					if (closest[i] == null) {
						closest[i] = v;
					}
					else if (v.pose.s < closest[i].pose.s) {
						closest[i] = v;
					}
				}
			}
		}
		
		int laneToAdd = (leftLaneAutonomousOnly && isAutonomous || !leftLaneAutonomousOnly) ? (int)(Math.random()*lane_count) : (int)(Math.random()*(lane_count-1))+1;
		
		//If the last vehicle in the selected lane is far enough ahead, add a new vehicle at the same speed
		if (closest[laneToAdd]==null || closest[laneToAdd].pose.s > laneAddingThreshold) {
			Vehicle w = new Vehicle(laneToAdd, this, isAutonomous);
			if (closest[laneToAdd]!=null) {
				w.pose.v = closest[laneToAdd].pose.v;
			}
			
			else {
				w.pose.v = w.v_max;
			}
			w.pose.s = 0;
			w.pose.t = lanes[(int)w.pose.lane].t(w.getS());
			w.pose.s_left = ((int)w.pose.lane-1 >= 0) ? Optional.of(lanes[(int)w.pose.lane-1].s(w.pose.t)) : Optional.empty();
			w.pose.s_right = ((int)w.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)w.pose.lane+1].s(w.pose.t)) : Optional.empty();
			w.pose.r = lanes[(int)w.pose.lane].r(w.getS());
			vehicles.add(w);
		}
		
	}
	
	public void populateVehicles() {
		double length = lanes[0].length;
		for (int i = 1; i < lane_count; i++) {
			length = Math.min(length, lanes[i].length);
		}
		double s = 0;
		while (s < length) {
			boolean isAutonomous = Math.random() < autonomousPercentage;
			int laneToAdd = (leftLaneAutonomousOnly && isAutonomous || !leftLaneAutonomousOnly) ? (int)(Math.random()*lane_count) : (int)(Math.random()*(lane_count-1))+1;
			Vehicle w = new Vehicle(laneToAdd, this, isAutonomous);
			w.pose.s = s;
			w.pose.t = lanes[(int)w.pose.lane].t(w.getS());
			w.pose.s_left = ((int)w.pose.lane-1 >= 0) ? Optional.of(lanes[(int)w.pose.lane-1].s(w.pose.t)) : Optional.empty();
			w.pose.s_right = ((int)w.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)w.pose.lane+1].s(w.pose.t)) : Optional.empty();
			w.pose.r = lanes[(int)w.pose.lane].r(w.getS());
			w.pose.v = (1-Math.random()*.1)*w.v_max*starting_speed;
			w.setWaypoints();
			vehicles.add(w);
			s+=(1000.0/density) + (Math.random()-.5)*0.1*(1000.0/density);
		}
		
//		Vehicle w = new Vehicle(0, this, true);
//		w.pose.t = 1.5;
//		w.pose.s = lanes[(int)w.pose.lane].s(w.pose.t);
//		w.pose.s_left = ((int)w.pose.lane-1 >= 0) ? Optional.of(lanes[(int)w.pose.lane-1].s(w.pose.t)) : Optional.empty();
//		w.pose.s_right = ((int)w.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)w.pose.lane+1].s(w.pose.t)) : Optional.empty();
//		w.pose.r = lanes[(int)w.pose.lane].r(w.getS());
//		w.pose.v = 0;
//		w.v_max = 0;
//		w.setWaypoints();
//		vehicles.add(w);
//		
//		w = new Vehicle(0, this, true);
//		w.pose.t = 0;
//		w.pose.s = lanes[(int)w.pose.lane].s(w.pose.t);
//		w.pose.s_left = ((int)w.pose.lane-1 >= 0) ? Optional.of(lanes[(int)w.pose.lane-1].s(w.pose.t)) : Optional.empty();
//		w.pose.s_right = ((int)w.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)w.pose.lane+1].s(w.pose.t)) : Optional.empty();
//		w.pose.r = lanes[(int)w.pose.lane].r(w.getS());
//		w.pose.v = w.v_max;
//		w.setWaypoints();
//		vehicles.add(w);
//		
//		w = new Vehicle(0, this, true);
//		w.pose.t = .1;
//		w.pose.s = lanes[(int)w.pose.lane].s(w.pose.t);
//		w.pose.s_left = ((int)w.pose.lane-1 >= 0) ? Optional.of(lanes[(int)w.pose.lane-1].s(w.pose.t)) : Optional.empty();
//		w.pose.s_right = ((int)w.pose.lane+1 < lane_count) ? Optional.of(lanes[(int)w.pose.lane+1].s(w.pose.t)) : Optional.empty();
//		w.pose.r = lanes[(int)w.pose.lane].r(w.getS());
//		w.pose.v = w.v_max;
//		w.setWaypoints();
//		vehicles.add(w);
		
		updateSurrounding();
		
//		for (Vehicle v: vehicles) {
//			ArrayList<Vehicle> all = new ArrayList<Vehicle>(Arrays.asList(new Vehicle[]{v, v.front, v.back, v.rightFront, v.rightBack, v.leftFront, v.leftBack}));
//			System.out.print(v.pose.lane + ": ");
//			for (Vehicle w: all) {
//				System.out.print((w==null ? null : w.ID) + ", ");
//			}
//			System.out.println();
//		}
		
		for (Vehicle v: vehicles) {
			v.lastPose = v.pose.copy();
		}
		
	}
}
