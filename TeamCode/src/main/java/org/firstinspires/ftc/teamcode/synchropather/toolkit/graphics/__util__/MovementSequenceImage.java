package org.firstinspires.ftc.teamcode.synchropather.toolkit.graphics.__util__;

import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.util.ArrayList;

import javax.swing.JComponent;

@SuppressWarnings("serial")
public class MovementSequenceImage extends JComponent {
	
	private Synchronizer synchronizer;
	private double elapsedTime;
	private static double[] WORLD_ORIGIN = CanvasConstants.WORLD_ORIGIN;
	private static double PIXEL_PER_INCH = CanvasConstants.PIXEL_PER_INCH;
	
	public MovementSequenceImage(Synchronizer synchronizer) {
		this.synchronizer = synchronizer;
	}
	
	public void setMovementSequence(Synchronizer synchronizer) {
		this.synchronizer = synchronizer;
	}
	
	public void setElapsedTime(double elapsedTime) {
		this.elapsedTime = elapsedTime;
	}
		  
    public void paint(Graphics g)
    {
		double startOfFrame = System.currentTimeMillis();

        Graphics2D g2 = (Graphics2D) g;
        g2.setStroke(new BasicStroke(5));

		double duration = synchronizer.getDuration();

		double margin = 0.35;

		drawFrom(g2,
				0,
				duration,
				new BasicStroke(12),
				new Color(34, 187, 14, 8),
				new BasicStroke(5),
				new Color(34, 187, 14, 32)
		);

		drawFrom(g2,
				Math.max(0,elapsedTime-margin),
				Math.min(duration,elapsedTime+margin),
				new BasicStroke(12),
				new Color(34, 187, 14, 100),
				new BasicStroke(5),
				new Color(34, 187, 14, 200)
		);
    }

	/**
	 * Draws the robot's translational path from the given start to end time.
	 * @param g2
	 * @param startTime
	 * @param endTime
	 * @param bgStroke
	 * @param bgColor
	 * @param frStroke
	 * @param frColor
	 */
	private void drawFrom(Graphics2D g2, double startTime, double endTime, BasicStroke bgStroke, Color bgColor, BasicStroke frStroke, Color frColor) {

		TranslationState prev = (TranslationState) synchronizer.getState(MovementType.TRANSLATION, startTime);
		ArrayList<Integer> xPoly = new ArrayList<>(), yPoly = new ArrayList<>();
		xPoly.add((int)(prev.getX() * PIXEL_PER_INCH + WORLD_ORIGIN[0]));
		yPoly.add((int)(-prev.getY() * PIXEL_PER_INCH + WORLD_ORIGIN[1]));

		double dt = Math.max(0.05, DriveConstants.delta_t);
		double prevTheta = startTime;
		for (double count = startTime+dt; count < endTime; count += dt) {
			TranslationState current = (TranslationState) synchronizer.getState(MovementType.TRANSLATION, count);
			double currentTheta = 0;
			if (count!=dt) {
				currentTheta = current.minus(prev).theta();
			}
			if (!(current.equals(prev) || count==startTime+dt || Math.abs(currentTheta-prevTheta) < 1e-2) || count+dt > endTime) {
				xPoly.add((int) Math.round(current.getX() * PIXEL_PER_INCH + WORLD_ORIGIN[0]));
				yPoly.add((int) Math.round(-current.getY() * PIXEL_PER_INCH + WORLD_ORIGIN[1]));
			}
			prev = current;
			prevTheta = currentTheta;
		}

		// close the polygon
		for (int i = xPoly.size()-2; i >= 0; i--) {
			xPoly.add(xPoly.get(i));
			yPoly.add(yPoly.get(i));
		}

		// draw polygon
		Polygon poly = new Polygon(xPoly.stream().mapToInt(i -> i).toArray(), yPoly.stream().mapToInt(i -> i).toArray(), xPoly.size());
		g2.setStroke(bgStroke);
		g2.setColor(bgColor);
		g2.drawPolygon(poly);
		g2.setStroke(frStroke);
		g2.setColor(frColor);
		g2.drawPolygon(poly);
	}
    
}
