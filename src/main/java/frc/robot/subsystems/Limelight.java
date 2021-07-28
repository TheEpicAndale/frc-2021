// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Limelight subsystem is a light that is lime green. If you look at it at a
 * certain angle, you will go blind, so read this code with caution.
 */
public class Limelight extends SubsystemBase {
    private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");

    private final NetworkTableEntry txEntry = limeTable.getEntry("tx");
    private final NetworkTableEntry tyEntry = limeTable.getEntry("ty");
    private final NetworkTableEntry tvEntry = limeTable.getEntry("tv");
    private final NetworkTableEntry taEntry = limeTable.getEntry("ta");

    public double tx;
    public double ty;
    public boolean tv;
    public double ta;

    public double previousAngle = Double.MAX_VALUE;

    public double previousTx = tx;
    public double deltaTx = 0;

    @Override
    public void periodic() {
        tx = txEntry.getDouble(0);
        ty = tyEntry.getDouble(0);
        tv = tvEntry.getDouble(0) == 1;
        ta = taEntry.getDouble(0);
        deltaTx = Math.abs(tx - previousTx);
        previousTx = tx;
    }

    public double getEstimatedDistance() {
        // Formula: tan(a1 + a2) = (h2 - h1) / d OR d = some constant / ta (area as %)
        return 3 / ta/*
                      * (Constants.TARGET_HEIGHT - Constants.MOUNTING_HEIGHT) /
                      * Math.tan(Constants.MOUNTING_ANGLE + ty)
                      */;
    }

    public double getEstimatedRPM() {
        // Use estimated distance to get RPM - to be implemented via map/formula when we
        // get values
        /*
         * Dist | RPM 10ft | 6000 12ft | 6500 14ft | 7200 What is 11? Estimate would be
         * 6250 (Would use estimate if clear formula cannot be found for curve)
         */
        return getEstimatedDistance();
    }
}