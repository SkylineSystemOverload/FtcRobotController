// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

// imports ---------------------------------------------------------------------
import android.view.WindowAnimationFrameStats;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.InstantiableUserConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutonomousInstructions;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import java.util.ArrayList;

// op mode class ---------------------------------------------------------------
@Autonomous(name = "LearningAutonomous", group = "Test")
public class LearningAutonomous extends LinearOpMode {
    public void ShootyShootyBangBang(){
        robot.motor7.setPower(1);
    }
    public void TacticalReload(){
        robot.servo1.setPosition(1);
    }
    public void TacticalReloadTheReload() {
        robot.servo1.setPosition(.5);
    }
    public void Kill(){
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor7.setPower(0);
    }
    public long TimeElapsed(){
        long elapsedTime = System.currentTimeMillis() - startTime;
        return elapsedTime;
    }

    public int Add(int numOne, int numTwo) {
        return numOne + numTwo;
    }

    // op mode global vars -----------------------------------------------------
    //Calls the RobotHardware class
    RobotHardware robot = new RobotHardware();
    AutonomousInstructions instructions = new AutonomousInstructions();
    long startTime;

    // initialization ----------------------------------------------------------
    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes hardware when init is pressed on the phone
        robot.init(hardwareMap);

        Add(10, 20);



        // define drive motors
        ArrayList<DcMotor> driveMotors = new ArrayList<>(Arrays.asList(robot.motor1, robot.motor2, robot.motor3, robot.motor4));

        // adding autonomous instructions
        instructions.AddTestDrive(300, driveMotors, 30, instructions.strafeRight);
        instructions.AddTestDrive(10000, driveMotors, 30, instructions.driveForward);

        /*
        instructions.AddTestDrive( 0, driveMotors, 60, instructions.driveForward);
        instructions.AddSeqMotorPowerInstruction(400, robot.motor7,.45);
        instructions.AddSeqServoInstruction(2000, robot.servo1, 1, false);
        instructions.AddSeqServoInstruction(2000,robot.servo1,0, false);
    */

        // The program will wait for the start button to continue.
        waitForStart();
        startTime = System.currentTimeMillis();
        // autonomous loop (when auton is started) -----------------------------
        while (opModeIsActive()) { // only run when opmodeisactive

            long elapsedTime = TimeElapsed();

            instructions.HandleInstructions(elapsedTime, 0.0);

        }

        instructions.KillAllInstructions();
        Kill();
    }
}