// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

// imports ---------------------------------------------------------------------
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

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import java.util.ArrayList;

// op mode class ---------------------------------------------------------------
@Autonomous(name = "Test", group = "Test")
public class TestBlue extends LinearOpMode {
    // instruction keys --------------------------------------------------------
    // these keys make typing them into the add instruction methods easier to autocomplete
    final int driveForward = 0;
    final int driveBackward = 1;
    final int strafeLeft = 2;
    final int strafeRight = 3;
    final int turnLeft = 4;
    final int turnRight = 5;

    // op mode global vars -----------------------------------------------------
    //Calls the RobotHardware class
    AndysRobotHardware robot = new AndysRobotHardware();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    final double power = .5; // default power is never modified (used for the driving methods)
    final double speed = .5;
    double globalAngle, correction, rotation;
    private ElapsedTime     runtime = new ElapsedTime();

    //Calls the PIDHardware class
    PIDHardware pidRotate, pidDrive;

    // instatiates the autonomous instruction handler class
    AutonomousInstructions Instructions = new AutonomousInstructions();

    // auton instruction vars --------------------------------------------------
    // get start time to zero


    // movement methods --------------------------------------------------------
    //Declares some methods to compress and reduce tediousness of writing repetitive code.
    //Every time the method DriveForward() is called,  it will do the instructions within the method
    private void ResetDriverEncoders() {
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void SetModeAndSpeedForDrivers() {
        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motor1.setPower(speed);
        robot.motor2.setPower(speed);
        robot.motor3.setPower(speed);
        robot.motor4.setPower(speed);
    }

    public void DriveForward(int distance) // key is "driveForward"
    {

        ResetDriverEncoders();

        // now set the new target and set their velocities
        robot.motor1.setTargetPosition(distance);
        robot.motor2.setTargetPosition(distance);
        robot.motor3.setTargetPosition(distance);
        robot.motor4.setTargetPosition(distance);

        SetModeAndSpeedForDrivers();
    }

    //Same as DriveForward() but in reverse
    public void DriveBackward(int distance) // key is "driveBackward"
    {
        ResetDriverEncoders();

        // now set the new target and set their velocities
        robot.motor1.setTargetPosition(-distance);
        robot.motor2.setTargetPosition(-distance);
        robot.motor3.setTargetPosition(-distance);
        robot.motor4.setTargetPosition(-distance);

        SetModeAndSpeedForDrivers();
    }

    public void StrafeLeft(int distance) // key is "strafeLeft"
    {
        // WORKS YAYADON T OTHER THINSG LIFKE THIS
        ResetDriverEncoders();

        // now set the new target and set their velocities
        robot.motor1.setTargetPosition(-distance);
        robot.motor2.setTargetPosition(distance);
        robot.motor3.setTargetPosition(distance);
        robot.motor4.setTargetPosition(-distance);

        SetModeAndSpeedForDrivers();
    }

    public void StrafeRight( int distance) // key is "strafeRight"
    {
        ResetDriverEncoders();

        // now set the new target and set their velocities
        robot.motor1.setTargetPosition(distance);
        robot.motor2.setTargetPosition(-distance);
        robot.motor3.setTargetPosition(-distance);
        robot.motor4.setTargetPosition(distance);

        SetModeAndSpeedForDrivers();
    }

    public void TurnLeft(double power) // key is "turnLeft"
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(power + correction);
    }

    public void TurnRight(double power) // key is "turnRight"
    {
        robot.motor1.setPower(power - correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(-power + correction);
    }

    //Stops all 4 motors
    public void StopDriving() // key is "stopDriving"
    {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }

    public void EndOPMode() {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor5.setPower(0);
        robot.motor6.setPower(0);
        robot.motor7.setPower(0);
    }

    // initialization ----------------------------------------------------------
    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes hardware when init is pressed on the phone
        robot.init(hardwareMap);

        //Makes new methods for naming simplification purposes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDHardware(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDHardware(.05, 0, 0);

        //Gives info about what the IMU is doing on the phone
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // When the stop button isn't pushed and the gyro (IMU) isn't calibrated, wait (! means not). This is a loop.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            //do nothing for 50 milliseconds
            sleep(50);
            //idle(); allows the program to perform other necessary tasks in between iterations of the loop.
            idle();
        }
        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        //Proceeds with the following code as long as the mode is active (returns false when stop button is pushed or power is disconnected).
        //The difference between (opModeIsActive()) and (isStopRequested()) is the first requires the play (not init) button to be pushed
        //the latter does not (this is just my guess).

        // instructions --------------------------------------------------------
        // update information on the driver station phone screen
        telemetry.setAutoClear(false); // when telemetry.update() is called it clears the screen, this portion helps show if the instructions are loaded
        telemetry.addData("Instructions", "Initializing");
        telemetry.update();

        ArrayList<DcMotor> driveMotors = new ArrayList<DcMotor>(Arrays.asList(robot.motor1, robot.motor2, robot.motor3, robot.motor4));

        // sequential instructions
        Instructions.AddSeqDrivingInstruction(100, driveMotors, 12, Instructions.driveForward);

        // update information on the driver station phone screen
        telemetry.addData("Loaded Instructions", "Success");
        telemetry.addData("Mode", "Ready");
        telemetry.update();
        telemetry.setAutoClear(true);

        // keep track of the start time to zero in on the actual time in the op mode
        boolean started = false;
        long startTime = 0;
        long elapsedTime;

        // The program will wait for the start button to continue.
        waitForStart();

        // autonomous loop (when auton is started) -----------------------------
        while (opModeIsActive() && Instructions.StillRunning()) { // only run when opmodeisactive and auto ends the op mode when instructions run out

            // simple switch that sets the time as soon as op mode is started (activates once at the beginning of loop)
            if (!started) {
                started = true;
                startTime = System.currentTimeMillis();
            }

            elapsedTime = System.currentTimeMillis() - startTime;

            // Use PID with imu input to drive in a straight line.
            //renames pidDrive.performPID(getAngle()) to correction for simple nomenclature.
            correction = pidDrive.performPID(getAngle());

            // update all instructions
            Instructions.HandleInstructions(elapsedTime, correction);

            //Displays the realtime heading information on the phone.
            /*
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.addData("Total Instructions Left", GetInstructionsAmount());
            telemetry.addData("Imu Heading", lastAngles.firstAngle);
            telemetry.addData("Global Heading", globalAngle);
            telemetry.addData("Correction", correction);
            telemetry.addData("Turn Rotation", rotation);*/

            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.addData("Instructions", Instructions.InstructionLeft());
            telemetry.addData("Encoder 1", robot.motor1.getCurrentPosition());
            telemetry.addData("Encoder 2", robot.motor2.getCurrentPosition());
            telemetry.addData("Encoder 3", robot.motor3.getCurrentPosition());
            telemetry.addData("Encoder 4", robot.motor4.getCurrentPosition());
            telemetry.addData("Encoder 7", robot.motor7.getCurrentPosition());
            telemetry.update();

        }
        EndOPMode();
    }

    // Resets the cumulative angle tracking to zero.
    private void resetAngle() {
        //ZYX
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //Get current cumulative angle rotation from last reset.
    //return Angle in degrees. + = left, - = right.
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //The change (delta) angle is whatever the angle was first minus our current angle.
        //lastAngles is new Orientation() as declared near the beginning.
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;


        //If the angle goes below -180, start subtracting the change in the angle (so -180 is the greatest change in the angle, -360 would be 0).
        //Returns a positive value (left) if the robot turns more than 180 deg right.
        if (deltaAngle < -180)
            deltaAngle += 360;
            //If the angle goes above 180, start subtracting the change in the angle (so 180 is the greatest change in the angle, 360 would be 0).
            //Returns a negative value (right) if the robot turns more than 180 deg left.
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the deltaAngle to the globalAngle (which was set to 0 at the start, but will change as the robot moves).
        globalAngle += deltaAngle;

        //Sets the current lastAngles to angles, which is different from what lastAngles was to start.
        lastAngles = angles;

        return globalAngle;
    }


    /*/**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        // rotate until turn is completed.
        if (degrees < 0) { // right turn
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                TurnRight();
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                TurnLeft();
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                TurnLeft();
            } while (opModeIsActive() && !pidRotate.onTarget());


        // turn the motors off.
        StopDriving(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }*/
}