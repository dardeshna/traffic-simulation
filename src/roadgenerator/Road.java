package roadgenerator;

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

import javax.swing.JFrame;
import javax.swing.JPanel;

import vehicle.Vehicle;


public class Road extends JFrame {
	
	
	public static final double kEps = 1e-5;

	private static final long serialVersionUID = 1L;
	public static final int WIDTH = 800;  //Width of frame
	public static final int HEIGHT = WIDTH/2;  //Height of frame
	public static final int CONVERSION_FACTOR = WIDTH/8;

	
	public ArrayList<Vehicle> vehicles = new ArrayList<Vehicle>();
	public int lane_count = 4;
	public double lane_spacing = 3;  //meters
	public double laneAddingThreadshold = 10; //meters
	public CubicSegmentPath[] lanes = new CubicSegmentPath[lane_count];
	
	public double t;
	public int counter;
	public double dt = 0.01;
	
	public Road(ControlPoint[] points) {		
		for (int i = 0; i < lanes.length; i++) {
			lanes[i] = new CubicSegmentPath(points, i*lane_spacing);
		}
		
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
				
				double ds = 1e-1;
				
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
				for (Vehicle v: vehicles) {
					double r[] = v.getR();
					graphics.setColor(Color.BLUE);
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
	/**
	 * Updates the vehicle pose and surrounding vehicles
	 * @param dt timestep
	 */
	public void updateVehicles(double dt) {
		ArrayList<Vehicle> finished = new ArrayList<Vehicle>();
		
		//Update vehicle pose
		for (Vehicle v: vehicles) {
			v.update(dt);
			v.pose.r = lanes[(int)v.pose.lane].r(v.getS());
			v.pose.t = lanes[(int)v.pose.lane].t(v.getS());
			if (v.getS() >= lanes[(int)v.pose.lane].getPathLength()) {
				finished.add(v);
				v.used = true;
			}
			
		}
		
		//Remove finished vehicles
		for (Vehicle v: finished) {
			vehicles.remove(v);
		}
		
		//Update surrounding vehicles for each vehicle
		for (Vehicle v: vehicles) {
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
		
		//Update dependent pose elements for each vehicle (actual gap, relative velocity)
		for (Vehicle v: vehicles) {
			v.updateDependent();
		}
		
	}
	
	public void updateGraphics() {
		this.repaint();
	}
	
	/**
	 * Main update function for the simulation
	 */
	public void update() {
		updateVehicles(dt);
		if (counter > 100) {
			addVehicles();
			counter = 0;
		}
		else {
			counter++;
		}
		updateGraphics();
		t += dt;
		
	}

	/** 
	 * Adds new vehicles to the road
	 */
	private void addVehicles() {
		Vehicle[] closest = new Vehicle[lane_count];
		
		//Obtains the last vehicle in each lain
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
		
		int laneToAdd = (int)(Math.random()*lane_count);
		
		//If the last vehicle in the selected lane is far enough ahead, add a new vehicle at the same speed
		if (closest[laneToAdd]==null || closest[laneToAdd].pose.s > laneAddingThreadshold) {
			Vehicle w = new Vehicle(laneToAdd);
			if (closest[laneToAdd]!=null) {
				w.lastPose.v = closest[laneToAdd].pose.v;
				w.pose.v = closest[laneToAdd].pose.v;
			}
			
			else {
				double v = w.v_max;
				w.lastPose.v = v;
				w.pose.v = v;
			}
			w.pose.r = lanes[(int)w.pose.lane].r(w.getS());
			w.pose.t = lanes[(int)w.pose.lane].t(w.getS());
			vehicles.add(w);
		}
		
	}
}
