package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous

public class AutoBlue extends LinearOpMode {
    public void runOpMode() {
        StoneTracker stoneTracker = new StoneTracker();
        stoneTracker.init(hardwareMap);
        stoneTracker.activate();
    
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Slide slide = new Slide(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Hand hand = new Hand(hardwareMap);
        Brain brain = new Brain(hardwareMap, hand, slide, drivetrain, elbow, stoneTracker, telemetry);
        waitForStart();
        brain.autonomousBlue();
    }
}
