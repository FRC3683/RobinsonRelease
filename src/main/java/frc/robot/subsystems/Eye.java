/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.MathUtils;
import frc.robot.config.EyeConstants;

/**
 * Add your docs here.
 */
public class Eye {
	private static Eye instance;

	/**
	 * This function implements a simple method of generating driving and steering
	 * commands based on the tracking data from a Rasperry Pi writing to a network
	 * table under the name Eye.
	 */
	public static Eye getInstance() {
		if (instance == null) {
			instance = new Eye();
		}
		return instance;
	}

	private NetworkTable nt;
    private double ty;
    private double tx;
    private boolean tv;
    private boolean looking;
	private MedianFilter distanceFilter;
	private MedianFilter txFilter;

	private Eye() {
		nt = NetworkTableInstance.getDefault().getTable("limelight");
        ty = 0.0;
        ty = 0.0;
        tv = false;
		distanceFilter = new MedianFilter(28);
		txFilter = new MedianFilter(5);
	}

	public double getDistance() {
		double theta = ty + EyeConstants.limelightMountAngle;
		double tanty = Math.tan(Math.toRadians(theta));		
		double deltaH = EyeConstants.goalHeight - EyeConstants.limelightMountHeight;
		return deltaH/tanty;
	}

	public double getHeadingCorrection() {
		return tx;
	}

	public boolean onTarget() {
		double tol = MathUtils.remap(80, 400, getDistance(), 6.5, 2.5);
		return tv && MathUtils.closeEnough(tx, 0, tol);
	}

	public boolean onBaselockTarget(){
		return MathUtils.closeEnough(tx, 0, EyeConstants.baselockTolerance);
	}

	public boolean onAimlockTarget(){
		return MathUtils.closeEnough(tx, 0, EyeConstants.aimlockTolerance);
	}

	public boolean validTarget() {
		return tv;
	}

	public void look() {
		nt.getEntry("ledMode").setNumber(3.0);
		nt.getEntry("camMode").setNumber(0.0);
        looking = true;
	}

	public void close() {
		nt.getEntry("ledMode").setNumber(1.0);
		nt.getEntry("camMode").setNumber(1.0);
        looking = false;
        ty = 0.0;
        tx = 0.0;
        tv = false;
	}

    public void update(){
        if(looking){
            ty = distanceFilter.calculate(nt.getEntry("ty").getDouble(0));
            tx = txFilter.calculate(nt.getEntry("tx").getDouble(0));
            tv = nt.getEntry("tv").getDouble(0) > 0.5;
        }
    }
}
