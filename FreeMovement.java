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

@TeleOp(name="FreeMovement", group="Linear Opmode")

public class FreeMovement extends LinearOpMode {
    boolean table = false;
    boolean find = false;
    
    boolean looking = false;
    boolean found = false;
    
    @Override
    public void runOpMode() {
        //Init StoneTracker
        StoneTracker stoneTracker = new StoneTracker();
        stoneTracker.init(hardwareMap);
        stoneTracker.activate();

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Slide slide = new Slide(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Hand hand = new Hand(hardwareMap);
        Brain brain = new Brain(hardwareMap, hand, slide, 
                drivetrain, elbow, stoneTracker, telemetry);
        
        waitForStart();

        while (opModeIsActive()) {
            //*******************
            // Brain
            //*******************
            
            if(gamepad2.left_trigger > 0) {
                brain.autonomousBlue();
            }
            
            if (gamepad2.right_trigger == 0) {
                looking = true;
            }
            if(gamepad2.right_trigger > 0 && looking == true && found == false) {
                found = brain.approachSkyStoneBlue();
                if (found == true) {
                    looking = false;
                    found = false;
                }
            }
            if (gamepad1.right_trigger > 0) {
                brain.captureStoneForDriver();
            }
            if (gamepad2.y) {
                brain.toRelativeBlockPosition(1);
            }
            
            //*******************
            // Hand
            //*******************
            if(gamepad2.right_bumper) {
                hand.close();
            }
            if(gamepad2.left_bumper) {
                hand.open();
            }
            if(gamepad2.b) {
                hand.closeStart();
            }
            telemetry.addData("GetPos", hand.handServo.getPosition());
            
            //*******************
            // Slide
            //*******************
            telemetry.addData("Slide", slide.extend(-gamepad2.right_stick_y*0.25));
            
            //*******************
            // Elbow
            //*******************
            telemetry.addData("Elbow", elbow.move(-gamepad2.left_stick_y));
            
            if(gamepad2.a) {
                telemetry.addData("Going", "Up");
                elbow.calibrate();
                slide.calibrate();
            }
            
            if(gamepad2.x) {
                elbow.toZero();
            }
            
            //*******************
            // Drivetrain
            //*******************
            if(!find && !gamepad1.right_bumper) {
                drivetrain.driveWithLimit(
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_stick_x
                );
            }
            if (!find && gamepad1.right_bumper) {
                drivetrain.driveWithLimit(
                        -gamepad1.left_stick_y*0.25,
                        gamepad1.right_stick_x*0.25,
                        gamepad1.left_stick_x*0.25
                );
            }
            // Direction
            while(gamepad1.y) {drivetrain.setHeading(0);}
            while(gamepad1.x) {drivetrain.setHeading(90);} // +angle is left
            while(gamepad1.b) {drivetrain.setHeading(-90);} // -angle is right
            while(gamepad1.a) {drivetrain.setHeading(180);}

            telemetry.update();
       }
    }
}
