package org.firstinspires.ftc.teamcode.synchropather.toolkit.graphics.__util__;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;

import java.awt.Color;
import java.awt.Font;
import java.text.DecimalFormat;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * Object that utilizes Java AWT to visualize a MovementSequence in a pop-up window.
 */
public class Visualizer {

	private Synchronizer synchronizer;
	private boolean running, paused, requestedStop;
	private JFrame frame;
	private JLabel x, y, h, timeLabel, fps;
	private double time, elapsedTime, lastTime, deltaTime;
	private TranslationState translationVelocity, translationState;
	private RotationState rotationVelocity, rotationState;
	private Movement currentMovement;
	private RobotImage robotImage;
	private MovementSequenceImage splineImage;
	private final double timeFactor;
	private final int fontSize;

	/**
	 * Creates a new Visualizer object with the given Synchronizer.
	 * @param synchronizer
	 */
	public Visualizer(Synchronizer synchronizer) {
		this(synchronizer, 1, 25);
	}
	
	/**
	 * Creates a new Visualizer object with the given Synchronizer.
	 * @param synchronizer
	 * @param timeFactor between 0 and 1
	 */
	public Visualizer(Synchronizer synchronizer, double timeFactor) {
		this(synchronizer, timeFactor, 25);
	}

	/**
	 * Creates a new Visualizer object with the given Synchronizer.
	 * @param synchronizer
	 * @param timeFactor between 0 and 1
	 * @param fontSize
	 */
	public Visualizer(Synchronizer synchronizer, double timeFactor, int fontSize) {
		this.timeFactor = bound(timeFactor, 0, 1);
		this.synchronizer = synchronizer;
		this.running = false;
		this.paused = false;
		this.requestedStop = false;
		this.time = synchronizer.getDuration();
		this.fontSize = fontSize;
	}
	
	/**
	 * @return the total runtime per cycle.
	 */
	public double getTime() {
		return time;
	}
	
	/**
	 * @return the JFrame object of this Visualizer.
	 */
	public JFrame getFrame() {
		return frame;
	}
	
	/**
	 * Launches the window and begins this Visualizer.
	 */
	public void start() {
		if (running) throw new RuntimeException("Simulation is already running!");
		running = true;
		
		// init frame
        frame = new JFrame("SynchroPather");
        frame.setSize(782, 805);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setIconImage(new ImageIcon("synchropather_v1.0/toolkit/graphics/__util__/DRIVE.png").getImage());
        
        // creating coordinate box
        JPanel textArea = new JPanel();   
        textArea.setLayout(null);
        textArea.setBackground(new Color(224, 224, 224, 96));  
        textArea.setBounds(0, 0, 11*fontSize, (int)(4.75*fontSize));
        frame.add(textArea);
        
        timeLabel = new JLabel("-s/-s");
        timeLabel.setVerticalAlignment(JLabel.TOP);
        timeLabel.setHorizontalAlignment(JLabel.LEFT);
        timeLabel.setSize(500, 50);
        timeLabel.setLocation(10, 0);
        timeLabel.setForeground(Color.gray);
        timeLabel.setFont(new Font(Font.MONOSPACED, Font.BOLD, fontSize));
        textArea.add(timeLabel);
        
        x = new JLabel("X: -");
        x.setVerticalAlignment(JLabel.TOP);
        x.setHorizontalAlignment(JLabel.LEFT);
        x.setSize(500, 50);
        x.setLocation(10, fontSize);
        x.setForeground(Color.gray);
        x.setFont(new Font(Font.MONOSPACED, Font.BOLD, fontSize));
        textArea.add(x);
        
        y = new JLabel("Y: -");
        y.setVerticalAlignment(JLabel.TOP);
        y.setHorizontalAlignment(JLabel.LEFT);
        y.setSize(500, 50);
        y.setLocation(10, 2*fontSize);
        y.setForeground(Color.gray);
        y.setFont(new Font(Font.MONOSPACED, Font.BOLD, fontSize));
        textArea.add(y);

		h = new JLabel("H: -");
		h.setVerticalAlignment(JLabel.TOP);
		h.setHorizontalAlignment(JLabel.LEFT);
		h.setSize(500, 50);
		h.setLocation(10, 3*fontSize);
		h.setForeground(Color.gray);
		h.setFont(new Font(Font.MONOSPACED, Font.BOLD, fontSize));
		textArea.add(h);

		fps = new JLabel("- FPS");
		fps.setVerticalAlignment(JLabel.TOP);
		fps.setHorizontalAlignment(JLabel.LEFT);
		fps.setSize(500, 50);
		fps.setLocation(670, 0);
		fps.setForeground(Color.gray);
		fps.setFont(new Font(Font.MONOSPACED, Font.BOLD, fontSize));
		frame.add(fps);

        robotImage = new RobotImage();
        splineImage = new MovementSequenceImage(synchronizer);
        robotImage.setSplineImage(splineImage);
        frame.add(robotImage);
        
        frame.setVisible(true);
        System.out.println("Simulation Started");
		
		lastTime = System.nanoTime() / 1e9d;
		deltaTime = 0;
		
	}
	
	/**
	 * Handles the next frame of this Visualizer if it is running. Use as the condition for a while loop.
	 * @return whether or not stop has been requested.
	 */
	public boolean loop() {
		if (requestedStop) {
			running = false;
			requestedStop = false;
			
			frame.setVisible(false);
			frame.dispose();
			
	        System.out.println("Simulation Ended");
	        
			return false;
		}

		if (running) {
			double currentTime = System.nanoTime() / 1e9d;
			deltaTime = currentTime - lastTime;
			elapsedTime += deltaTime * timeFactor;
			if (elapsedTime > time)
				elapsedTime = 0;

			// get velocities
			translationVelocity = (TranslationState) synchronizer.getVelocity(MovementType.TRANSLATION, elapsedTime);
			rotationVelocity = (RotationState) synchronizer.getVelocity(MovementType.ROTATION, elapsedTime);
			rotationVelocity = RotationState.zero.minus(rotationVelocity);

			// get current states
			translationState = (TranslationState) synchronizer.getState(MovementType.TRANSLATION, elapsedTime);
			rotationState = (RotationState) synchronizer.getState(MovementType.ROTATION, elapsedTime);

			DecimalFormat df = new DecimalFormat("0.0");
			DecimalFormat tl = new DecimalFormat("0.000");
			timeLabel.setText(String.format("%ss/%ss", tl.format(elapsedTime), tl.format(time)));
			x.setText(String.format("X:%5s", df.format(translationState.getX())));
			y.setText(String.format("Y:%5s", df.format(translationState.getY())));
			h.setText(String.format("H:%6s", df.format(rotationState.getHeading()*180/Math.PI)));
			fps.setText(String.format("%s FPS", Math.round(1/deltaTime)));
						
			robotImage.setPose(translationState, rotationState, elapsedTime);

	        frame.repaint();
	        lastTime = currentTime;
	        
	        while (paused) lastTime = System.currentTimeMillis() / 1000.0;
		} 
		else throw new RuntimeException("loop() called but simulation is not running!");
		
		return true;
	}
	
	/**
	 * Pauses this Visualizer if it is running.
	 */
	public void pause() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		if (!paused)
	        System.out.println("Simulation Paused");
		paused = true;
	}

	/**
	 * Un-pauses this Visualizer if it is running.
	 */
	public void unpause() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		if (paused)
	        System.out.println("Simulation Unpaused");
		paused = false;
	}
	
	/**
	 * Requests this Visualizer to stop and close the JFrame window.
	 */
	public void stop() {
		requestedStop = true;
	}
	
	/**
	 * @return the current elapsed time of this Visualizer if it is running.
	 */
	public double getElapsedTime() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		return elapsedTime;
	}
	
	/**
	 * @return the time since the last time loop() was called if this Visualizer is running.
	 */
	public double getDeltaTime() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		return deltaTime;
	}

	/**
	 * @return the current translationState of this Visualizer if it is running.
	 */
	public TranslationState getTranslationState() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		return translationState;
	}

	/**
	 * @return the current rotationState of this Visualizer if it is running.
	 */
	public RotationState getRotationState() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		return rotationState;
	}

	/**
	 * @return the current translationVelocity of this Visualizer if it is running.
	 */
	public TranslationState getTranslationVelocity() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		return translationVelocity;
	}

	/**
	 * @return the current rotationVelocity of this Visualizer if it is running.
	 */
	public RotationState getRotationVelocity() {
		if (!running) throw new RuntimeException("Simulation is not running!");
		return rotationVelocity;
	}

	/**
	 * Clips the input x between a given lower and upper bound.
	 * @param x
	 * @param lower
	 * @param upper
	 * @return the clipped value of x.
	 */
	private static double bound(double x, double lower, double upper) {
		return Math.max(lower, Math.min(upper, x));
	}

	/**
	 * Normalizes a given angle to [-pi,pi) radians.
	 * @param degrees the given angle in radians.
	 * @return the normalized angle in radians.
	 */
	private double normalizeAngle(double degrees) {
	    double angle = degrees;
	    while (angle <= -Math.PI) //TODO: opMode.opModeIsActive() && 
	        angle += 2*Math.PI;
	    while (angle > Math.PI)
	        angle -= 2*Math.PI;
	    return angle;
	}
	
}
