package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Brain {
    boolean foundationMoved = true;
    Drivetrain drivetrain;
    Hand hand;
    Slide slide;
    Elbow elbow;
    StoneTracker stoneTracker;
    Telemetry telemetry;
    LinearOpMode opMode;
    int currentBlockLevel = -1;
    int[] slidePositions = {1600, 1330, 1650, 2450, 0};
    int[] elbowPositions = {1200, 540, -170, -780, 0};
    
    public Brain(HardwareMap hardwareMap, 
            Hand aHand, Slide aSlide, Drivetrain aDrivetrain, 
            Elbow anElbow, StoneTracker aStoneTracker, Telemetry aTelemetry) {
        drivetrain = aDrivetrain;
        hand = aHand;
        slide = aSlide;
        elbow = anElbow;
        stoneTracker = aStoneTracker;
        telemetry = aTelemetry;
    }

    public boolean approachSkyStoneBlue() {
        telemetry.addData("", stoneTracker.scan());
        telemetry.addData("", stoneTracker.toString());
        telemetry.addData("", stoneTracker.closestSkyStoneHeight());
        telemetry.addData("", stoneTracker.closestSkyStoneAngle());

        int targetHeight = 340;
        double targetAngle = -2;
        double currentHeight = 0;
        double currentAngle = 0;
        double drive = 0;
        double strafe = 0;

        currentHeight = stoneTracker.closestSkyStoneHeight();
        currentAngle = stoneTracker.closestSkyStoneAngle();

        // Adjust for distance
        if (currentHeight == 0) {
            drive = 0;
        } else if(currentHeight < targetHeight - 200) {
            drive = 0.5;
        } else if(currentHeight < targetHeight - 10) {
            drive = 0.25;
        } else if(currentHeight > targetHeight + 10) {
            drive = -0.25;
        }
        //**********************************
        drive = 0; // do not move forward
        //**********************************
        
        // Adjust for angle
        // If no SkyStone, strafe left.
        if (currentHeight == 0) {
            strafe = -0.15;
        } else if (currentAngle < targetAngle - 1) {
            strafe = -0.1;
        } else if (currentAngle > targetAngle + 1) {
            strafe = 0.1;
        }

        if ((drive != 0) || (strafe != 0)) {            
            drivetrain.drive(drive, drivetrain.headingAdjust(0), strafe);
            return false;
        } else {
            drivetrain.drive(0, 0, 0);
            return true;
        }
    }
    public boolean approachSkyStoneRed() {
        telemetry.addData("", stoneTracker.scan());
        telemetry.addData("", stoneTracker.toString());
        telemetry.addData("", stoneTracker.closestSkyStoneHeight());
        telemetry.addData("", stoneTracker.closestSkyStoneAngle());

        int targetHeight = 340;
        double targetAngle = -2;
        double currentHeight = 0;
        double currentAngle = 0;
        double drive = 0;
        double strafe = 0;

        currentHeight = stoneTracker.closestSkyStoneHeight();
        currentAngle = stoneTracker.closestSkyStoneAngle();

        // Adjust for distance
        if (currentHeight == 0) {
            drive = 0;
        } else if(currentHeight < targetHeight - 200) {
            drive = 0.5;
        } else if(currentHeight < targetHeight - 10) {
            drive = 0.25;
        } else if(currentHeight > targetHeight + 10) {
            drive = -0.25;
        }
        //**********************************
        drive = 0; // do not move forward
        //**********************************
        
        // Adjust for angle
        // If no SkyStone, strafe left.
        if (currentHeight == 0) {
            strafe = 0.15;
        } else if (currentAngle < targetAngle - 1) {
            strafe = -0.1;
        } else if (currentAngle > targetAngle + 1) {
            strafe = 0.1;
        }

        if ((drive != 0) || (strafe != 0)) {            
            drivetrain.drive(drive, drivetrain.headingAdjust(0), strafe);
            return false;
        } else {
            drivetrain.drive(0, 0, 0);
            return true;
        }
    }

    public void toRelativeBlockPosition(int input) {
        if (input > 0 && currentBlockLevel < blockPositions.length-1) {
            currentBlockLevel++;
        }

        if (input < 0 && currentBlockLevel > 0) {
            currentBlockLevel--;
        }

        if (currentBlockLevel >= 0) {
            slide.moveToPosition(slidePositions[currentBlockLevel]);
            elbow.moveToPosition(elbowPositions[currentBlockLevel]);
        }
    }

    public void captureStoneForDriver() {
        hand.open();
        drivetrain.forwardDistance(-2, 0.3);
        slide.moveToPosition(2100);
        elbow.moveToPosition(1575);
        hand.close();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        elbow.toDrivingPos();
    }

    public void captureStoneForAutonomous() {
        hand.open();
        elbow.almostFullyDown();
        drivetrain.forwardDistance(14, 0.3);
        elbow.fullyDown();
        hand.close();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        elbow.toZero();
    }

    public void autonomousBlue() {
        // startup
        hand.close();
        slide.fullyExtend();
        drivetrain.forwardDistance(13, 0.5);
        
        // get stone
        while(!approachSkyStoneBlue()) {}
        captureStoneForAutonomous();
        drivetrain.forwardDistance(-2, 0.3);
        
        // turn, drive, and drop
        while(!drivetrain.setHeading(90)); // +angle is left
        elbow.toDrivingPos();
        drivetrain.forwardToColorNoStop("blue", .9, 90); // to center line

        // if foundation in original location
        drivetrain.forwardDistance(17, .9);
        drivetrain.strafeDistance(23, 0.9);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(200);} catch(InterruptedException ule) {}
        drivetrain.strafeDistance(-23, 0.9);
        //*/

        /*/ if foundation moved to building site
        drivetrain.forwardDistance(19, 0.5);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        //*/

        /*/ if foundation in front of robot
        drivetrain.forwardDistance(13, 0.5);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        //*/
        
        // back to center line
        drivetrain.forwardToColorNoStop("blue", -0.8, 90);
        drivetrain.forwardDistance(3, 0.5);
        elbow.toZero();
        hand.close();

        if (false) { // back to starting point
            try {Thread.sleep(5000);} catch(InterruptedException ule) {}
            drivetrain.forwardDistance(-42, 0.5);
            while(!drivetrain.setHeading(0));
            drivetrain.forwardDistance(-26, 0.5);
            slide.fullyRetract();
            hand.closeStart();
        }

    }
    public void autonomousRed() {
        // startup
        hand.close();
        slide.fullyExtend();
        drivetrain.forwardDistance(13, 0.5);
        
        // get stone
        while(!approachSkyStoneRed()) {}
        captureStoneForAutonomous();
        drivetrain.forwardDistance(-2, 0.3);
        
        // turn, drive, and drop
        while(!drivetrain.setHeading(-90)); // +angle is left
        elbow.toDrivingPos();
        drivetrain.forwardToColorNoStop("red", .9, -90); // to center line

        // if foundation in original location
        drivetrain.forwardDistance(17, .9);
        drivetrain.strafeDistance(-23, 0.9);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(200);} catch(InterruptedException ule) {}
        drivetrain.strafeDistance(23, 0.9);
        //*/

        /*/ if foundation moved to building site
        drivetrain.forwardDistance(19, 0.5);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        //*/
        
        /*/ if foundation in front of robot
        drivetrain.forwardDistance(13, 0.5);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        //*/
        
        // back to center line
        drivetrain.forwardToColorNoStop("red", -0.8, -90);
        drivetrain.forwardDistance(3, 0.5);
        elbow.toZero();
        hand.close();

        if (false) { // back to starting point
            try {Thread.sleep(5000);} catch(InterruptedException ule) {}
            drivetrain.forwardDistance(-42, 0.5);
            while(!drivetrain.setHeading(0));
            drivetrain.forwardDistance(-26, 0.5);
            slide.fullyRetract();
            hand.closeStart();
        }

    }
}
