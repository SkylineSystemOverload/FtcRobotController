// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

// imports ---------------------------------------------------------------------
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Array;
import java.util.ConcurrentModificationException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import java.util.ArrayList;

// op mode class ---------------------------------------------------------------
@Autonomous(name = "AndysOPMode", group = "Test")
public class AndysOPMode extends LinearOpMode {
    // instruction keys --------------------------------------------------------
    // these keys make typing them into the add instruction methods easier to autocomplete
    final int driveForward = 0;
    final int driveBackward = 1;
    final int strafeLeft = 2;
    final int strafeRight = 3;
    final int turnLeft = 4;
    final int turnRight = 5;

    // op mode instruction classes --------------------------------------------

    // driving instructions calls the specified driving methods
    public class DrivingInstruction {
        // private class variables
        private final long startTime;
        private final long stopTime;
        private boolean dead = false; // instruction handler checks if this is false, if false then instruction handler removes the instruction from the arraylist
        private final int method;

        // initialization constructor
        public DrivingInstruction(long startTime, long duration, int methodKey) {
            // stores start and end time
            // stores the instructions method key
            // has a method for updating
            // stores a boolean that tells if the instruction is now useless (when the instruction is done)
            this.startTime = startTime;
            this.stopTime = this.startTime + duration;
            this.method = methodKey;
        }

        // called in the instruction handler
        public void Update(long elapsedTime) {
            // checks if the instruction can be done
            if (elapsedTime >= this.startTime && elapsedTime < this.stopTime) {
                // do the method using switch case
                switch (this.method) {
                    case driveForward :
                        DriveForward();
                    case driveBackward :
                        DriveBackward();
                    case strafeLeft :
                        StrafeLeft();
                    case strafeRight :
                        StrafeRight();
                    case turnLeft :
                        TurnLeft();
                    case turnRight :
                        TurnRight();
                    default : // default in the off case someone enters the wrong key
                        StopDriving();
                }
            }
            else if (elapsedTime > this.stopTime){
                // declare instruction dead
                this.dead = true;
                StopDriving();
            }
        }
    }

    // motor instructions interact with individual motors and sets their powers
    public static class MotorInstruction {
        // private class variable
        private final long startTime;
        private final double power;
        private boolean dead = false;
        private final DcMotor motorID;

        // initialization constructor
        public MotorInstruction(long startTime, double power, DcMotor motorID ) {
            // stores start and end time
            // stores the instructions method key
            // has a method for updating
            // stores a boolean that tells if the instruction is now useless (when the instruction is done)
            this.startTime = startTime;
            this.power = power;
            this.motorID = motorID;
        }
        // called in the instruction handler
        public void Update(long elapsedTime) {
            // checks if the instruction can be done
            if (elapsedTime >= this.startTime) {
                // set the motor power
                this.motorID.setPower(this.power);
                this.dead = true;
            }
        }
    }

    // servo instructions interact with individual servos and sets their positions
    public static class ServoInstruction {
        // private vars
        private final long startTime;
        private boolean dead = false;
        private final double position;
        private final Servo servoID;

        // initialization constructor
        public ServoInstruction(long startTime, double position, Servo servoID) {
            // stores start and end time
            // stores the instructions method key
            // has a method for updating
            // stores a boolean that tells if the instruction is now useless (when the instruction is done)
            this.startTime = startTime;
            this.position = position;
            this.servoID = servoID;
        }
        // called in the instruction handler
        public void Update(long elapsedTime) {
            // checks if the instruction can be done
            if (elapsedTime >= this.startTime) {
                // set the servo's position
                this.servoID.setPosition(this.position);
                this.dead = true;
            }
        }
    }

    // op mode global vars -----------------------------------------------------
    //Calls the RobotHardware class
    RobotHardware robot = new RobotHardware();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    final double power = .5; // default power is never modified (used for the driving methods)
    double globalAngle, correction, rotation;
    //static final double   COUNTS_PER_MOTOR_REV = 560 ;
    //static final double   MAX_ENCODER_TICKS_PER_MIN = (300 * COUNTS_PER_MOTOR_REV);
    //double Speed = ;
    private ElapsedTime     runtime = new ElapsedTime();

    //Calls the PIDHardware class
    PIDHardware pidRotate, pidDrive;

    // auton instruction vars --------------------------------------------------
    // get start time to zero

    // arraylists that stores the instructions name and the time (milliseconds) for it to run
    ArrayList<DrivingInstruction> drivingInstructions = new ArrayList<>();
    ArrayList<MotorInstruction> motorInstructions = new ArrayList<>();
    ArrayList<ServoInstruction> servoInstructions = new ArrayList<>();

    // movement methods --------------------------------------------------------
    //Declares some methods to compress and reduce tediousness of writing repetitive code.
    //Every time the method DriveForward() is called,  it will do the instructions within the method
    public void DriveForward() // key is "driveForward"
    {
        /*//This is set up so that the motors' speed ramps up and ramps down while driving
        //Most of this probably shouldn't go in the DriveForward() method but I don't know where to put it.

        //Gets a motor's current power. Recommended to start with 0 power.
        //NOTE* If the motors were starting at different speeds, you'd have to dynamically adjust for that.
        //Figuring out a way to dynamically adjust the power so that the robot slows down even more to allow time for the other motors to catch up is beyond me.
        //It would create multiple different curves for each motor and would add another factor to consider when adjusting time intervals.
        double currentPower = robot.motor1.getPower();

        double targetPower = power; //power, as defined earlier in the code, will be named targetPower
        runtime.reset(); //resets the time

        if (targetPower > currentPower) { //LOOP THIS CONDITION, BUT FIND ANOTHER WAY TO PREVENT THE ADDITION FROM LOOPING
            double newPower = currentPower + 3*Math.pow(runtime.milliseconds(),2); //The main equation: adds a power-curve to whatever the current power was
        }
        else if (targetPower < currentPower) { //LOOP THIS CONDITION, BUT FIND ANOTHER WAY TO PREVENT THE ADDITION FROM LOOPING
            double newPower = currentPower + -3*Math.pow(runtime.milliseconds(),2); //The main equation: adds a power-curve to whatever the current power was
        }
        else {
            double newPower = targetPower;
        }

        if (newPower > targetPower && targetPower > currentPower) {
            newPower = targetPower; //caps the power added by the graph to the power we set
        }

        if (newPower < targetPower && targetPower < currentPower) {
            newPower = targetPower; //caps the power added by the graph to the power we set
        }
        robot.motor1.setPower(newPower - correction); //sets the motors power
        robot.motor2.setPower(newPower + correction);
        robot.motor3.setPower(newPower - correction);
        robot.motor4.setPower(newPower + correction);*/

        robot.motor1.setPower(power - correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(power + correction);
    }

    //Same as DriveForward() but in reverse
    public void DriveBackward() // key is "driveBackward"
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(-power + correction);
    }

    public void StrafeLeft() // key is "strafeLeft"
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(power - correction);
        robot.motor4.setPower(-power + correction);
    }

    public void StrafeRight() // key is "strafeRight"
    {
        robot.motor1.setPower(power - correction);
        robot.motor2.setPower(-power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(power + correction);
    }

    public void TurnLeft() // key is "turnLeft"
    {
        robot.motor1.setPower(-power - correction);
        robot.motor2.setPower(power + correction);
        robot.motor3.setPower(-power - correction);
        robot.motor4.setPower(power + correction);
    }

    public void TurnRight() // key is "turnRight"
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

    // instruction handler in the auton loop
    public void InstructionHandler(long elapsedTime) throws ConcurrentModificationException {

        ArrayList<DrivingInstruction> deadDrivingInstructions = new ArrayList<>();
        ArrayList<MotorInstruction> deadMotorInstructions = new ArrayList<>();
        ArrayList<ServoInstruction> deadServoInstructions = new ArrayList<>();


        // iterate through the instructions of each list
        for (DrivingInstruction i : drivingInstructions) {
            i.Update(elapsedTime);
            if (i.dead) {
                deadDrivingInstructions.add(i);
            }
        }
        for (MotorInstruction i : motorInstructions) {
            i.Update(elapsedTime);
            if (i.dead) {
                deadMotorInstructions.add(i);
            }
        }
        for (ServoInstruction i : servoInstructions) {
            i.Update(elapsedTime);
            if (i.dead) {
                deadServoInstructions.add(i);
            }
        }

        // iterate through and remove dead instructions
        for (DrivingInstruction i : deadDrivingInstructions) {
            if (i.dead) {
                drivingInstructions.remove(i);
            }
        }
        for (MotorInstruction i : deadMotorInstructions) {
            if (i.dead) {
                motorInstructions.remove(i);
            }
        }
        for (ServoInstruction i : deadServoInstructions) {
            i.Update(elapsedTime);
            if (i.dead) {
                servoInstructions.remove(i);
            }
        }
    }

    // easy methods to add instructions
    public void AddDrivingInstruction(long startTime, long duration, int methodKey) {
        drivingInstructions.add(new DrivingInstruction(startTime, duration, methodKey));
    }

    public void AddMotorInstruction(long startTime, double power, DcMotor motorID) {
        motorInstructions.add(new MotorInstruction(startTime, power, motorID));
    }

    public void AddServoInstruction(long startTime, double position, Servo servoID) {
        servoInstructions.add(new ServoInstruction(startTime, position, servoID));
    }

    // get the total amount of instructions left
    public int GetInstructionsAmount() {
        return drivingInstructions.size() + motorInstructions.size() + servoInstructions.size();
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

        // The program will wait for the start button to continue.
        waitForStart();

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

        // driving instructions
        AddDrivingInstruction(1000, 500, strafeRight); //Strafe Right
        AddDrivingInstruction(2000, 3750, driveForward); //DriveForward
        AddDrivingInstruction(6250, 250, strafeRight); //Strafe Right
        AddDrivingInstruction(10500, 400, strafeLeft); //Strafe Left
        AddDrivingInstruction(12900, 400, strafeLeft); //Strafe Left
        AddDrivingInstruction(15400, 500, driveForward); //DriveForward

        // update information on the driver station phone screen
        telemetry.addData("Driving Instructions", "Success");
        telemetry.update();

        // motor instructions
        AddMotorInstruction(0, -0.5, robot.motor5); //Reverse Collector
        AddMotorInstruction(500, 0, robot.motor5); //Turn Off Collector
        AddMotorInstruction(6500, 0.55, robot.motor7); //Turn On Shooter
        AddMotorInstruction(15400, 0, robot.motor7); //Turn Off Shooter

        // update information on the driver station phone screen
        telemetry.addData("Motor Instructions", "Success");
        telemetry.update();

        // servo instructions
        AddServoInstruction(8500, 1.2, robot.servo1);//Shoot
        AddServoInstruction(9500, .5, robot.servo1);//Reset
        AddServoInstruction(11400, 1.2, robot.servo1);//Shoot
        AddServoInstruction(12400, .5, robot.servo1);//Reset
        AddServoInstruction(13900, 1.2, robot.servo1);//Shoot
        AddServoInstruction(14900, .5, robot.servo1);//Reset

        // update information on the driver station phone screen
        telemetry.addData("Servo Instructions", "Success");
        telemetry.addData("Mode", "Ready");
        telemetry.update();
        telemetry.setAutoClear(true);

        // keep track of the start time to zero in on the actual time in the op mode
        boolean started = false;
        long startTime = 0;
        long elapsedTime = 0;

        // autonomous loop (when auton is started) -----------------------------
        while (opModeIsActive() && GetInstructionsAmount() > 0) { // only run when opmodeisactive and auto ends the op mode when instructions run out

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
            InstructionHandler(elapsedTime);

            //Displays the realtime heading information on the phone.
            telemetry.addData("Elapsed Time", elapsedTime);
            telemetry.addData("Total Instructions Left", GetInstructionsAmount());
            telemetry.addData("Imu Heading", lastAngles.firstAngle);
            telemetry.addData("Global Heading", globalAngle);
            telemetry.addData("Correction", correction);
            telemetry.addData("Turn Rotation", rotation);
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

        // update information on the driver station phone screen
        telemetry.addData("Driving Instructions", "Success");
        telemetry.update();

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


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
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

        if (degrees < 0) {
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
        StopDriving();

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}