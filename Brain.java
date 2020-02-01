package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.graphics.Color;


public class Brain {
    boolean foundationMoved = true;
    private NormalizedColorSensor colorSensor = null;
    Drivetrain drivetrain;
    Hand hand;
    Slide slide;
    Elbow elbow;
    StoneTracker stoneTracker;
    Telemetry telemetry;
    LinearOpMode opMode;
    int currentBlockLevel = 0;
    //int[] slidePositions = {1600, 1330, 1650, 2450, 0};
    //int[] elbowPositions = {1200, 540, -170, -780, 0};
    int[] slidePositions = {1000, 980, 1095, 1199, 0};
    int[] elbowPositions = {1078, 486, -374, -1244, 0};
    ElapsedTime timer = new ElapsedTime();
    
    
    public Brain(HardwareMap hardwareMap, 
            Hand aHand, Slide aSlide, Drivetrain aDrivetrain, 
            Elbow anElbow, StoneTracker aStoneTracker, Telemetry aTelemetry) {
        drivetrain = aDrivetrain;
        hand = aHand;
        slide = aSlide;
        elbow = anElbow;
        stoneTracker = aStoneTracker;
        telemetry = aTelemetry;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public boolean approachSkyStoneBlue() {
        telemetry.addData("", stoneTracker.scan());
        telemetry.addData("", stoneTracker.toString());
        telemetry.addData("", stoneTracker.closestSkyStoneHeight());
        telemetry.addData("", stoneTracker.closestSkyStoneAngle());

        int targetHeight = 340;
        double targetAngle = -4;
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
        double targetAngle = -4;
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
        if (input > 0 && currentBlockLevel < elbowPositions.length-1) {
            currentBlockLevel++;
        }

        if (input < 0 && currentBlockLevel > 0) {
            currentBlockLevel--;
        }

        if (currentBlockLevel >= 0) {
            elbow.move(0);
            slide.extend(0);
            drivetrain.drive(0, 0, 0);
            slide.moveToPosition(slidePositions[currentBlockLevel]);
            elbow.moveToPosition(elbowPositions[currentBlockLevel]);
        }
    }

    public void captureStoneForDriver() {
        elbow.move(0);
        slide.extend(0);
        drivetrain.drive(0, 0, 0);
        hand.open();
        drivetrain.forwardDistance(-3, 0.3);
        slide.moveToPosition(2000);
        elbow.moveToPosition(1575);
        hand.close();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        elbow.toDrivingPos();
    }

    public void captureStoneForAutonomous() {
        hand.open();
        elbow.almostFullyDown();
        drivetrain.forwardDistance(13, 0.3);
        elbow.fullyDown();
        hand.close();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        elbow.toZero();
    }

    public void autonomousBlue() {
        // startup
        hand.close();
        drivetrain.forwardDistance(13, 0.5);
        
        // get stone
        //while(!approachSkyStoneBlue()) {}
        //
        double maxTime = 6*1000;
        double startTime = timer.milliseconds();
        while(!approachSkyStoneBlue()) {
            double currentTime = timer.milliseconds();
            double elapsed = currentTime - startTime;
            if (elapsed>maxTime) {
                drivetrain.strafeDistance(10, 0.5);
                drivetrain.forwardDistance(12, 0.5);
                while(!drivetrain.setHeading(90));
                toRelativeBlockPosition(-1);
                //drive.strafeDistance(, 0.2);?
                drivetrain.forwardToColorNoStop("blue", 0.5, 90);
                drivetrain.forwardDistance(-2, 0.5);
                //drivetrain.forwardToColorNoStop("red", 0.5, -90);
                drivetrain.drive(0, 0, 0);
                return;
            }
        }
        ///
        
        
        slide.fullyExtend();
        captureStoneForAutonomous();
        drivetrain.forwardDistance(-1, 0.3);

        // turn, drive, and drop
        while(!drivetrain.setHeading(90)); // +angle is left
        elbow.toDrivingPos();
        drivetrain.forwardToColorNoStop("blue", .9, 90); // to center line

        /*// if foundation in original location
        drivetrain.forwardDistance(18, .9);
        //while(drivetrain.setHeading(90));
        toRelativeBlockPosition(1);
        drivetrain.strafeDistance(28, 0.9);
        drivetrain.forwardDistance(4, .3);
        //try {Thread.sleep(500);} catch(InterruptedException ule) {}
        toRelativeBlockPosition(-1);
        hand.open();
        try {Thread.sleep(200);} catch(InterruptedException ule) {}
        //while(drivetrain.setHeading(90));
        drivetrain.strafeDistance(-28, 0.9);
        //*/

        /// if foundation moved to building site
        toRelativeBlockPosition(1);
        drivetrain.strafeDistance(-4, 0.5);
        drivetrain.forwardDistance(23, 0.5);
        toRelativeBlockPosition(-1);
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        //drivetrain.forwardDistance(-2, 0.5);
        drivetrain.strafeDistance(4, 0.5);
        ///

        /*/ if foundation in front of robot
        drivetrain.forwardDistance(13, 0.5);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        /*/
        
        // back to center line
        drivetrain.forwardToColorNoStop("blue", -0.8, 90);
        drivetrain.forwardDistance(3, 0.5);

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
        drivetrain.forwardDistance(13, 0.5);
        
        // get stone
        //while(!approachSkyStoneBlue()) {}
        //
        double maxTime = 6*1000;
        double startTime = timer.milliseconds();
        while(!approachSkyStoneRed()) {
            double currentTime = timer.milliseconds();
            double elapsed = currentTime - startTime;
            if (elapsed>maxTime) {
                drivetrain.strafeDistance(-10, 0.5);
                drivetrain.forwardDistance(12, 0.5);
                while(!drivetrain.setHeading(-90));
                toRelativeBlockPosition(-1);
                //drive.strafeDistance(, 0.2);?
                //drivetrain.forwardToColorNoStop("blue", 0.5, 90);
                drivetrain.forwardToColorNoStop("red", 0.5, -90);
                drivetrain.forwardDistance(-2, 0.5);
                drivetrain.drive(0, 0, 0);
                return;
            }
        }
        ///
        
        
        slide.fullyExtend();
        captureStoneForAutonomous();
        drivetrain.forwardDistance(-1, 0.3);

        // turn, drive, and drop
        while(!drivetrain.setHeading(-90)); // +angle is left
        elbow.toDrivingPos();
        drivetrain.forwardToColorNoStop("red", .9, -90); // to center line

        /// if foundation in original location
        drivetrain.forwardDistance(18, .9);
        //while(drivetrain.setHeading(-90));
        toRelativeBlockPosition(1);
        drivetrain.strafeDistance(-28, 0.9);
        drivetrain.forwardDistance(6, .3);
        //try {Thread.sleep(500);} catch(InterruptedException ule) {}
        toRelativeBlockPosition(-1);
        hand.open();
        //drivetrain.forwardDistance(-3, 0.5);
        try {Thread.sleep(200);} catch(InterruptedException ule) {}
        //while(drivetrain.setHeading(-90));
        drivetrain.strafeDistance(28, 0.9);
        //*/

        /*// if foundation moved to building site
        toRelativeBlockPosition(1);
        drivetrain.strafeDistance(4, 0.5);
        drivetrain.forwardDistance(23, 0.5);
        toRelativeBlockPosition(-1);
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        //drivetrain.forwardDistance(-2, 0.5);
        drivetrain.strafeDistance(-4, 0.5);
        //*/

        /*/ if foundation in front of robot
        drivetrain.forwardDistance(13, 0.5);
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        hand.open();
        try {Thread.sleep(500);} catch(InterruptedException ule) {}
        /*/
        
        // back to center line
        drivetrain.forwardToColorNoStop("red", -0.8, -90);
        drivetrain.forwardDistance(3, 0.5);

        if (false) { // back to starting point
            try {Thread.sleep(2500);} catch(InterruptedException ule) {}
            drivetrain.forwardDistance(-42, 0.5);
            while(!drivetrain.setHeading(0));
            drivetrain.forwardDistance(-26, 0.5);
            slide.fullyRetract();
            hand.closeStart();
        }

    }
}
