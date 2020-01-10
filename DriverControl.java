package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

@TeleOp(name="DriverControl", group="Linear Opmode")

public class DriverControl extends LinearOpMode {
    boolean table = false;
    boolean find = false;
    @Override
    public void runOpMode() {
        //Init StoneTracker
        StoneTracker stoneTracker = new StoneTracker();
//        stoneTracker.init(hardwareMap);
//        stoneTracker.activate();

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Slide slide = new Slide(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Hand hand = new Hand(hardwareMap);
        Brain brain = new Brain(hardwareMap, hand, slide, 
                drivetrain, elbow, stoneTracker, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            //*******************
            // Hand
            //*******************
            if(gamepad2.right_bumper) {
                hand.close();
            }
            if(gamepad2.left_bumper) {
                hand.open();
            }
            
            //*******************
            // Slide
            //*******************
            slide.extendRestricted(-gamepad2.right_stick_y);
            
            //*******************
            // Elbow
            //*******************
            elbow.moveRestricted(-gamepad2.left_stick_y);

            if (gamepad2.x) {elbow.toDrivingPos();}
            if (gamepad2.b) {elbow.toZero();}
            if (gamepad2.y) {brain.toRelativeBlockPosition(+1);}
            if (gamepad2.a) {brain.toRelativeBlockPosition(-1);}
            
            //*******************
            // Drivetrain
            //*******************
            //Disable drivetrain with all buttons for when on table
            if(!find && !gamepad1.right_bumper) {
                drivetrain.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
                );
            }
            if (!find && gamepad1.right_bumper) {
                drivetrain.drive(
                    -gamepad1.left_stick_y*0.25,
                    gamepad1.right_stick_x*0.25,
                    gamepad1.left_stick_x*0.25
                );
            }
            //Only when y pressed
//            if(gamepad1.y && !(gamepad1.x || gamepad1.b || gamepad1.a)) {
//                find = true;
//                drivetrain.setHeading(0);
//            }
//            find = false;
            
            while(gamepad1.x) {drivetrain.setHeading(0);}
            while(gamepad1.a) {drivetrain.setHeading(90);} // +angle is left
            while(gamepad1.y) {drivetrain.setHeading(-90);} // -angle is right
            while(gamepad1.a) {drivetrain.setHeading(180);}

            //*******************
            // Automation
            //*******************
            if (gamepad1.right_trigger > 0.5) {
                brain.captureStoneForDriver();
            }
        }
    }
}
