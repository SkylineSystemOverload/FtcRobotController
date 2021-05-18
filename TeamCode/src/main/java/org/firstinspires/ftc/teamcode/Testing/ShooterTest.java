// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode.Testing;

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
@Autonomous(name = "ShooterTest", group = "Test")
public class ShooterTest extends LinearOpMode {

    // op mode global vars -----------------------------------------------------
    //Calls the RobotHardware class
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    // instatiates the autonomous instruction handler class
    AutonomousInstructions Instructions = new AutonomousInstructions();

    // used to end the whole op mode
    public void EndOPMode() {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor5.setPower(0);
        robot.motor6.setPower(0);
        robot.motor7.setPower(0);

        Instructions.KillAllInstructions();
    }

    // initialization ----------------------------------------------------------
    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes hardware when init is pressed on the phone
        robot.init(hardwareMap);

        // When the stop button isn't pushed and the gyro (IMU) isn't calibrated, wait (! means not). This is a loop.

        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        telemetry.addData("Mode", "running");
        telemetry.update();
        ;

        //Proceeds with the following code as long as the mode is active (returns false when stop button is pushed or power is disconnected).
        //The difference between (opModeIsActive()) and (isStopRequested()) is the first requires the play (not init) button to be pushed
        //the latter does not (this is just my guess).

        ArrayList<DcMotor> driveMotors = new ArrayList<>(Arrays.asList(robot.motor1, robot.motor2, robot.motor3, robot.motor4));
        boolean addedInstructions = false;

        Instructions.AddSeqMotorDistanceInstruction(100, robot.motor7, 60, false, false);

        // keep track of the start time to zero in on the actual time in the op mode
        boolean started = false;
        long startTime = 0;
        long elapsedTime;
        int position = 0;

        // The program will wait for the start button to continue.
        waitForStart();

        // autonomous loop (when auton is started) -----------------------------
        while (opModeIsActive()) { // only run when opmodeisactive and auto ends the op mode when instructions run out

            // simple switch that sets the time as soon as op mode is started and also performs wobble goal read (activates once at the beginning of loop)
            if (!started) {
                started = true;
                startTime = System.currentTimeMillis();
            }
            elapsedTime = System.currentTimeMillis() - startTime;

            // update all instructions
            Instructions.HandleInstructions(elapsedTime, 0);

            telemetry.addData("Instructions: ", Instructions.InstructionLeft());
            telemetry.addData("Rings: ", position);
            telemetry.addData("b: ", robot.motor7.getCurrentPosition());
            telemetry.update();

        }
        EndOPMode();
    }
}