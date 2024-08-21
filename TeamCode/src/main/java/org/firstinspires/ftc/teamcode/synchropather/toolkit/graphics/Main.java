package org.firstinspires.ftc.teamcode.synchropather.toolkit.graphics;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.LinearTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.toolkit.graphics.__util__.Visualizer;

public class Main {

	public static void main(String[] args) throws InterruptedException {

		// translation Plan
        CRSplineTranslation spline1 = new CRSplineTranslation(0,
				new TranslationState(-40.75,63.5),
				new TranslationState(-40.75,38),
				new TranslationState(43,36)
		);

        CRSplineTranslation spline2 = new CRSplineTranslation(spline1.getEndTime(),
				new TranslationState(43,36),
				new TranslationState(0,12),
				new TranslationState(-56,12)
		);

        CRSplineTranslation spline3 = new CRSplineTranslation(spline2.getEndTime(),
				new TranslationState(-56,12),
				new TranslationState(12,12),
				new TranslationState(43,36)
		);

        CRSplineTranslation spline4 = new CRSplineTranslation(spline3.getEndTime(),
				new TranslationState(43,36),
				new TranslationState(0,12),
				new TranslationState(-36,12),
				new TranslationState(-56,24)
		);

        CRSplineTranslation spline5 = new CRSplineTranslation(spline4.getEndTime(),
				new TranslationState(-56,24),
				new TranslationState(-36,12),
				new TranslationState(0,12),
				new TranslationState(43,36)
		);

		LinearTranslation line1 = new LinearTranslation(spline5.getEndTime(),
				new TranslationState(43,36),
				new TranslationState(43,12)
		);

		LinearTranslation line2 = new LinearTranslation(line1.getEndTime(),
				new TranslationState(43,12),
				new TranslationState(53,12)
		);

		LinearTranslation line3 = new LinearTranslation(line2.getEndTime(),
				new TranslationState(53,12),
				new TranslationState(0,0)
		);

		LinearTranslation line4 = new LinearTranslation(line3.getEndTime()+3,
				new TranslationState(0, 0),
				new TranslationState(48, 60)
		);

		LinearTranslation line5 = new LinearTranslation(line4.getEndTime(),
				new TranslationState(48, 60),
				new TranslationState(-48, 60)
		);

		LinearTranslation line6 = new LinearTranslation(line5.getEndTime(),
				new TranslationState(-48, 60),
				new TranslationState(-48, -60)
		);

		LinearTranslation line7 = new LinearTranslation(line6.getEndTime(),
				new TranslationState(-48, -60),
				new TranslationState(48, -60)
		);

		LinearTranslation line8 = new LinearTranslation(line7.getEndTime(),
				new TranslationState(48, -60),
				new TranslationState(0,0)
		);

        CRSplineTranslation returnToStart = new CRSplineTranslation(line8.getEndTime(),
				new TranslationState(0,0),
				new TranslationState(-36,12),
				new TranslationState(-40.75,63.5)
		);

		TranslationPlan translationPlan = new TranslationPlan(
				spline1,
				spline2,
				spline3,
				spline4,
				spline5,
				line1,
				line2,
				line3,
				line4,
				line5,
				line6,
				line7,
				line8,
				returnToStart
		);


		// rotation Plan
		LinearRotation rot1 = new LinearRotation(0,
				new RotationState(0),
				new RotationState(Math.toRadians(360))
		);

		RotationPlan rotationPlan = new RotationPlan(
				rot1
		);


        // put all the Plans into a Synchronizer
		Synchronizer synchronizer = new Synchronizer(
				translationPlan,
				rotationPlan
		);
		
		// put the MovementSequence into a visualizer object, with timeFactor between 0 and 1 representing the speed of the visualizer
		double timeFactor = 3;
		Visualizer visualizer = new Visualizer(synchronizer, timeFactor);
		
		// start visualizer
		visualizer.start();
		
		// main visualizer loop with an example telemetry function
		double targetFPS = 144;
		while (visualizer.loop()) {
//			generateTelemetry(visualizer, timeFactor);
			Thread.sleep((int)(1000/targetFPS));
		}

	}
	
	
	
	//////////////////
	// random stuff //
	//////////////////
	
	

	static double x = -40.75, y = 63.5, h = 0;
	static double[] maxAccel = {0,0,0}, maxAccelPos = {0,0}, prevVelocity = {0,0,0};
	
	private static void generateTelemetry(Visualizer visualizer, double timeFactor) {
		double dt = visualizer.getDeltaTime();
		TranslationState translationVelocity = visualizer.getTranslationVelocity();
		RotationState rotationVelocity = visualizer.getRotationVelocity();

		TranslationState translationState = visualizer.getTranslationState();
		RotationState rotationState = visualizer.getRotationState();

		double[] accel = {(translationVelocity.getX()-prevVelocity[0])/dt, (translationVelocity.getY()-prevVelocity[1])/dt, (rotationVelocity.getHeading()-prevVelocity[2])/dt};
		prevVelocity = new double[]{translationVelocity.getX(), translationVelocity.getY(), rotationVelocity.getHeading()};
		double m = Math.hypot(accel[0], accel[1]), theta = Math.atan2(accel[1], accel[0])*180d/Math.PI;
		accel[0] = m;
		accel[1] = theta;
		if (m > maxAccel[0]) {
			maxAccel[0] = m;
			maxAccel[1] = theta;
			maxAccelPos = new double[] {translationState.getX(), translationState.getY()};
		}
		if (Math.abs(accel[2]) > Math.abs(maxAccel[2]))
			maxAccel[2] = accel[2];
		
		double xv = translationVelocity.getX() * timeFactor;
		double yv = translationVelocity.getY() * timeFactor;
		double hv = rotationVelocity.getHeading() * 180d / Math.PI * timeFactor;
		x += dt * xv;
		y += dt * yv;
		h = normalizeAngle(h + dt * hv);
		System.out.println(String.format("\n\n\n\n\n\n\n\n\n\n\n\n\nRUNTIME [%ss]/[%ss] \n[position = getPose()] \n  X %s𝘪𝘯 \n  Y %s𝘪𝘯 \n  H %s° \n[position = ∫ν𝒹𝓉] \n  X %s𝘪𝘯 \n  Y %s𝘪𝘯 \n  H %s° \n[velocity] \n  X %s𝘪𝘯/𝘴 \n  Y %s𝘪𝘯/𝘴 \n  H %s°/𝘴 \n[accel] \n  m %s𝘪𝘯/𝘴/𝘴 \n  θ %s° \n  h %s𝘳𝘢𝘥/𝘴/𝘴 \n[maxAccel] \n  m %s𝘪𝘯/𝘴/𝘴 \n  θ %s° \n  @ (%s𝘪𝘯, %s𝘪𝘯)", 
				Math.round(visualizer.getElapsedTime()*10000)/10000.0, Math.round(visualizer.getTime()*10000)/10000.0,
				Math.round(translationState.getX()*100)/100.0, Math.round(translationState.getY()*100)/100.0, Math.round(rotationState.getHeading()*180/Math.PI*100)/100.0,
				Math.round(x*100)/100.0, Math.round(y*100)/100.0, Math.round(h*100)/100.0, 
				Math.round(xv*100)/100.0, Math.round(yv*100)/100.0, Math.round(hv*100)/100.0, 
				Math.round(accel[0]*100)/100.0, Math.round(accel[1]*100)/100.0, Math.round(accel[2]*100)/100.0,
				Math.round(maxAccel[0]*100)/100.0, Math.round(maxAccel[1]*100)/100.0,
				Math.round(maxAccelPos[0]*100)/100.0, Math.round(maxAccelPos[1]*100)/100.0
		));
		
	}
	
	/**
	 * Normalizes a given angle to [-pi,pi) radians.
	 * @param degrees the given angle in radians.
	 * @return the normalized angle in radians.
	 */
	private static double normalizeAngle(double degrees) {
	    double angle = degrees;
	    while (angle <= -180) //TODO: opMode.opModeIsActive() && 
	        angle += 360;
	    while (angle > 180)
	        angle -= 360;
	    return angle;
	}
	
}
