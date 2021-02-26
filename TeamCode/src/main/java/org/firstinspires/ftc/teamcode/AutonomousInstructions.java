package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.TickMeter;

import java.util.ArrayList;
import java.util.Arrays;

/*
instantiate this class into an object for use in any autonomous mode
 */
public class AutonomousInstructions {
    // variables for the autonomous instructions class object
    private final double speed = .75;
    private final int TICKS_PER_REVOLUTION = 1120;
    private final int WHEEL_DIAM = 4;
    private final double INCHES_PER_REVOLUTION = WHEEL_DIAM * Math.PI;
    private final double TICKS_PER_INCHES = TICKS_PER_REVOLUTION / INCHES_PER_REVOLUTION;
    private final double INCHES_PER_TICK = 1 / TICKS_PER_INCHES;
    private long drivingDelay = 0;

    private final int HD_HEX_MOTOR_MAX_RPM = 300;
    private final int MAX_TICKS_PER_SECOND = (HD_HEX_MOTOR_MAX_RPM / 60) * TICKS_PER_REVOLUTION;
    private final double MAX_INCHES_PER_SECOND = MAX_TICKS_PER_SECOND * INCHES_PER_TICK;
    private final int PROPORTIONATE_TICKS_PER_SECOND = (int)(MAX_TICKS_PER_SECOND * speed);
    private final double PROPORTIONATE_INCHES_PER_SECOND = PROPORTIONATE_TICKS_PER_SECOND * INCHES_PER_TICK;

    // these keys make typing them into the add instruction methods easier to autocomplete
    public final int driveForward = 0;
    public final int driveBackward = 1;
    public final int strafeLeft = 2;
    public final int strafeRight = 3;
    public final int turnLeft = 4;
    public final int turnRight = 5;

    // driving multipliers
    private final double driveMultiplier = 1.07;
    private final double strafeMultiplier = 1.25;

    // instruction list
    ArrayList<SequentialInstruction> seqInstructions = new ArrayList<>();
    ArrayList<TimerInstruction> timerInstructions = new ArrayList<>();

    // base instructions ---------------------------------------------------------------------------
    // base instruction class for sequential instructions
    private static class SequentialInstruction {
        // private variables for the sequential instructions
        protected final long delay;
        protected boolean dead = false;
        protected boolean started = false;
        protected boolean drive = false;

        /*
        initialization constructor
         */
        public SequentialInstruction(long delay) {
            this.delay = delay;
        }

        /*
        method for performing the sequential instruction
            since this is the base class and inheritors will have different actions, this method is a placeholder
         */
        public void Perform() {

        }

        /*
        method for updating the instruction if the instruction needs to be continuously performed
            since this is the base class and inheritors will have different actions, this method is a placeholder
         */
        public void Check(double placeholder) {

        }
    }

    // base instruction class for timer based instructions
    private static class TimerInstruction {
        // private variables for the
        protected final long startTime;
        protected boolean dead = false;

        /*
        initialization constructor
         */
        public TimerInstruction(long startTime) {this.startTime = startTime; }

        /*
        method for performing the timer instruction
            since this is the base class and inheritors will have different actions, this method is a placeholder
         */
        public void Perform(long currentTime) {}

        /*
        method for updating the instruction if the instruction needs to be continuously performed
            since this is the base class and inheritors will have different actions, this method is a placeholder
         */
        public void Check() {}
    }

    // sequential instructions ---------------------------------------------------------------------
    // sequential motor power instruction, doesn't need to a check method
    private class SequentialMotorPowerInstruction extends SequentialInstruction {
        // private variables used
        private final DcMotor motor;
        private final double power;

        /*
        initialization constructor for this class
         */
        public SequentialMotorPowerInstruction(long delay, DcMotor motor, double power) {
            // initialize the base class
            super(delay);
            // set private variables
            this.motor = motor;
            this.power = power;
        }

        /*
        perform method
            this instruction only needs to be called once to set the power of the motor
         */
        public void Perform() {
            // this instruction only sets the power of the motor and kills the instruction
            this.motor.setPower(this.power);
            this.dead = true;
        }
    }

    // sequential motor distance instruction
    private class SequentialMotorDistanceInstruction extends SequentialInstruction {
        // private variables used
        private final DcMotor motor;
        private final int distance;
        private final boolean wait; // this decides if the instruction handler waits for this instruction to finish to move on

        /*
        initialization constructor for this class
         */
        public SequentialMotorDistanceInstruction(long delay, DcMotor motor, double inches, boolean wait) {
            // initialize the base class
            super(delay);
            // set private variables
            this.motor = motor;
            this.distance = (int)Math.round(inches * TICKS_PER_INCHES);
            this.wait = wait;
        }

        /*
        perform method
            instruction resets the motor's run type and then sets the distance
         */
        public void Perform() {
            // reset the encoder's read value to 0
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set the distance for the encoder to travel to
            this.motor.setTargetPosition(this.distance);

            // reset the mode to get to the position
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // set the motor's power
            this.motor.setPower(speed);

            // kill if not waiting for this instruction to finish
            this.dead = !this.wait;

        }

        /*
        check method
            kill the instruction if waiting for the instruction to die
         */
        public void Check(double placeholder) {
            this.dead = !this.motor.isBusy();
        }
    }

    // sequential servo position instruction
    private class SequentialServoInstruction extends SequentialInstruction {
        // private variables used
        private final Servo servo;
        private final double position;
        private final boolean wait;

        /*
        initialization constructor for this class
         */
        public SequentialServoInstruction(long delay, Servo servo, double position, boolean wait) {
            // init base class
            super(delay);
            // set private variables
            this.servo = servo;
            this.position = position;
            this.wait = wait;
        }

        /*
        perform method
            instruction resets the motor's run type and then sets the distance
         */
        public void Perform() {
            this.servo.setPosition(this.position);
            this.dead = !this.wait;
        }

        /*
        check method
            instruction kills the instruction if waiting
         */
        public void Check(double placeholder) {
            if (this.servo.getPosition() == this.position) this.dead = true;
        }
    }

    // sequential drive to distance instruction
    private class SequentialDrivingInstruction extends SequentialInstruction {
        // private vars used
        private final ArrayList<DcMotor> driveMotors;
        private final int distance;
        private final int method;

        // used for speeding up and slowing down
        private double speedMultiplier;
        private long driveStartTime;
        private long slowStartTime;
        private boolean slowingDown = false;
        private final double distanceThreshold;

        private final double millisToSpeedUp = 500;
        private final double millisToSlowDown = 1000;
        private final double base = .15;
        private ArrayList<Integer> currentTicks = new ArrayList<>();
        private ArrayList<Integer> targetTicks = new ArrayList<>();

        // used for correction
        private boolean correcting = false;
        private final double correctionTolerance = .1;
        private final double correctingSpeed = .5;
        private final int encoderTolerance = 5;

        /*
        initialization constructor
         */
        public SequentialDrivingInstruction(long delay, ArrayList<DcMotor> driveMotors, double inches, int methodKey) {
            super(delay);
            this.driveMotors = driveMotors;
            if (methodKey == strafeLeft || methodKey == strafeRight) {
                inches *= strafeMultiplier;
            }
            else if (methodKey == driveForward || methodKey == driveBackward) {
                inches *= driveMultiplier;
            }
            this.distance = (int)Math.round(inches * TICKS_PER_INCHES);
            this.method = methodKey;
            this.drive = true;
            // make the threshold
            if (inches > 20) {
                this.distanceThreshold = this.distance * .7;
            }
            else {
                this.distanceThreshold = this.distance * .6;
            }
        }

        /*
        perform method
            instruction resets all encoders, sets target position and then drives
         */
        public void Perform() {
            // reset all drive motor encoders
            for (DcMotor motor: this.driveMotors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // determine the correct distance to input based on method key
            switch (this.method) {
                case driveForward:
                    for (DcMotor motor: this.driveMotors) {
                        motor.setTargetPosition(this.distance);
                    }
                    break;
                case driveBackward:
                    for (DcMotor motor: this.driveMotors) {
                        motor.setTargetPosition(-this.distance);
                    }
                    break;
                case strafeLeft:
                    this.driveMotors.get(0).setTargetPosition(-this.distance);
                    this.driveMotors.get(1).setTargetPosition(this.distance);
                    this.driveMotors.get(2).setTargetPosition(this.distance);
                    this.driveMotors.get(3).setTargetPosition(-this.distance);
                    break;
                case strafeRight:
                    this.driveMotors.get(0).setTargetPosition(this.distance);
                    this.driveMotors.get(1).setTargetPosition(-this.distance);
                    this.driveMotors.get(2).setTargetPosition(-this.distance);
                    this.driveMotors.get(3).setTargetPosition(this.distance);
                    break;
            }

            // set motors to run to the position
            for (DcMotor motor: this.driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            this.driveStartTime = System.currentTimeMillis();
        }

        /*
        check method
            uses imu to correct if not turning, if turning make sure turning correctly
         */
        public void Check(double correction) {
            if (this.method == turnLeft || this.method == turnRight) { // turning

            }
            else { // not turning
                // check if dead first
                if (!this.correcting && !this.driveMotors.get(0).isBusy() && !this.driveMotors.get(1).isBusy() && !this.driveMotors.get(2).isBusy() && !this.driveMotors.get(3).isBusy()) {
                    this.dead = true;
                }

                // apply correction otherwise
                if (!this.dead) {
                    // adjust multiplier to slow down
                    // find smallest distance traveled
                    int ticksTraveled = this.driveMotors.get(0).getCurrentPosition();
                    for (DcMotor motor: this.driveMotors) {
                        if (motor.getCurrentPosition() < ticksTraveled) {
                            ticksTraveled = motor.getCurrentPosition();
                        }
                    }

                    // slow down after hitting the threshold
                    if (ticksTraveled > this.distanceThreshold && !this.slowingDown) {
                        this.slowStartTime = System.currentTimeMillis();
                        this.slowingDown = true;
                    }

                    // adjust the multiplier
                    this.AdjustMultiplier();

                    // drive
                    this.driveMotors.get(0).setPower(speed * this.speedMultiplier);
                    this.driveMotors.get(1).setPower(speed * this.speedMultiplier);
                    this.driveMotors.get(2).setPower(speed * this.speedMultiplier);
                    this.driveMotors.get(3).setPower(speed * this.speedMultiplier);

                }
            }
            if (this.dead) {
                this.OnDeath();
            }
        }

        /*
        on death method
            kills all motors when done driving
         */
        private void OnDeath() {
            // iterate through drive motors to stop and reset encoders
            for (DcMotor motor: this.driveMotors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        // speed multiplier function
        private void AdjustMultiplier() {
            double value;

            /*final double twoSeocndsQuadraticConstantA = 2.5 * Math.pow(10, -7)
            if (this.slowingDown) {
                value = 1 - twoSeocndsQuadraticConstantA * Math.pow(System.currentTimeMillis() - this.slowStartTime, 2);
            }
            else {
                value = twoSeocndsQuadraticConstantA * Math.pow(System.currentTimeMillis() - this.driveStartTime, 2) + this.base;
            }*/

            // test
            if (this.slowingDown) {
                value = this.SlowDown();
            }
            else {
                value = this.SpeedUp();
            }

            this.speedMultiplier = value;

            if (this.speedMultiplier > 1) {
                this.speedMultiplier = 1;
            }
            else if (this.speedMultiplier < this.base) {
                this.speedMultiplier = this.base;
            }
        }

        // quadratic formulas for slowing down and speeding up
        private double SlowDown() {
            // define the three points
            final double x1 = -this.millisToSlowDown;
            final double y1 = this.base;
            final double x2 = 0;
            final double y2 = 1;
            final double x3 = this.millisToSlowDown;
            final double y3 = this.base;

            final double elapsedTime = System.currentTimeMillis() - this.slowStartTime;

            // return y
            return ((elapsedTime-x2) * (elapsedTime-x3)) / ((x1-x2) * (x1-x3)) * y1 +
                    ((elapsedTime-x1) * (elapsedTime-x3)) / ((x2-x1) * (x2-x3)) * y2 +
                    ((elapsedTime-x1) * (elapsedTime-x2)) / ((x3-x1) * (x3-x2)) * y3;
        }

        private double SpeedUp() {
            // define the three points
            // constants used for speeding up and slowing down
            final double x1 = -this.millisToSpeedUp;
            final double y1 = 1;
            final double x2 = 0;
            final double y2 = this.base;
            final double x3 = this.millisToSpeedUp;
            final double y3 = 1;

            final double elapsedTime = System.currentTimeMillis() - this.driveStartTime;

            // return y
            return ((elapsedTime-x2) * (elapsedTime-x3)) / ((x1-x2) * (x1-x3)) * y1 +
                    ((elapsedTime-x1) * (elapsedTime-x3)) / ((x2-x1) * (x2-x3)) * y2 +
                    ((elapsedTime-x1) * (elapsedTime-x2)) / ((x3-x1) * (x3-x2)) * y3;
        }
    }

    // timer instructions --------------------------------------------------------------------------


    // test instructions based on motor max speed --------------------------------------------------
    private class TestMotorDrive extends SequentialInstruction {
        private final ArrayList<DcMotor> driveMotors;
        private final double duration;
        private final int method;

        // used for speeding up and slowing down
        private double speedMultiplier;
        private long driveStartTime;
        private long slowStartTime;
        private boolean slowingDown = false;
        private final double timeThreshold;

        private final double millisToSpeedUp = 500;
        private final double millisToSlowDown = 1000;
        private final double base = .15;

        public TestMotorDrive(long delay, ArrayList<DcMotor> driveMotors, double inches, int methodKey) {
            super(delay);
            this.driveMotors = driveMotors;
            this.method = methodKey;
            // multiplier for the inches based on methodkey
            if (methodKey == driveForward || methodKey == driveBackward) {
                inches *= driveMultiplier;
            }
            else if (methodKey == strafeLeft || methodKey == strafeRight) {
                inches *= strafeMultiplier;
            }

            // convert inches into time
            double ticks = inches * TICKS_PER_INCHES;
            this.duration = ticks * MAX_TICKS_PER_SECOND * speed;

            // create time threshold
            this.timeThreshold = this.duration - this.millisToSlowDown;
        }

        public void Perform() {
            // iterate through each motor
            for (DcMotor motor: this.driveMotors) {
                // set to run with encoders (in case they weren't)
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // switch case to set the correct powers for the different driving methods
            switch (this.method) {
                case driveForward:
                    for (DcMotor motor: this.driveMotors) {
                        motor.setPower(speed);
                    }
                    break;
                case driveBackward:
                    for (DcMotor motor: this.driveMotors) {
                        motor.setPower(-speed);
                    }
                    break;
                case strafeLeft:
                    this.driveMotors.get(0).setPower(-speed);
                    this.driveMotors.get(1).setPower(speed);
                    this.driveMotors.get(2).setPower(speed);
                    this.driveMotors.get(3).setPower(-speed);
                    break;
                case strafeRight:
                    this.driveMotors.get(0).setPower(speed);
                    this.driveMotors.get(1).setPower(-speed);
                    this.driveMotors.get(2).setPower(-speed);
                    this.driveMotors.get(3).setPower(speed);
                    break;
            }

            // set the start time
            this.driveStartTime = System.currentTimeMillis();
        }

        public void Check(double correction) {
            final long elapsedTime = System.currentTimeMillis() - this.driveStartTime;
            // check if dead
            if (elapsedTime > this.duration) {
                this.dead = true;
            }

            // otherwise perform routine
            else {

                // test if we need to start slowing down
                if (elapsedTime > this.timeThreshold) {
                    this.slowStartTime = System.currentTimeMillis();
                    this.slowingDown = true;
                }

                // adjust multiplier
                this.AdjustMultiplier();

                // switch case since the sign of the speed value determines motor spin direction
                switch (this.method) {
                    case driveForward:
                        this.driveMotors.get(0).setPower((speed - correction));
                        this.driveMotors.get(1).setPower((speed + correction));
                        this.driveMotors.get(2).setPower((speed - correction));
                        this.driveMotors.get(3).setPower((speed + correction));
                        break;
                    case driveBackward:
                        this.driveMotors.get(0).setPower((-speed - correction));
                        this.driveMotors.get(1).setPower((-speed + correction));
                        this.driveMotors.get(2).setPower((-speed - correction));
                        this.driveMotors.get(3).setPower((-speed + correction));
                        break;
                    case strafeLeft:
                        this.driveMotors.get(0).setPower((-speed - correction));
                        this.driveMotors.get(1).setPower((speed + correction));
                        this.driveMotors.get(2).setPower((speed - correction));
                        this.driveMotors.get(3).setPower((-speed + correction));
                        break;
                    case strafeRight:
                        this.driveMotors.get(0).setPower((speed - correction));
                        this.driveMotors.get(1).setPower((-speed + correction));
                        this.driveMotors.get(2).setPower((-speed - correction));
                        this.driveMotors.get(3).setPower((speed + correction));
                        break;
                }

                // apply speed multiplier
                for (DcMotor motor: this.driveMotors) {
                    motor.setPower(motor.getPower() * this.speedMultiplier);
                }
            }
        }

        // speed multiplier function
        private void AdjustMultiplier() {
            double value;

            /*final double twoSeocndsQuadraticConstantA = 2.5 * Math.pow(10, -7)
            if (this.slowingDown) {
                value = 1 - twoSeocndsQuadraticConstantA * Math.pow(System.currentTimeMillis() - this.slowStartTime, 2);
            }
            else {
                value = twoSeocndsQuadraticConstantA * Math.pow(System.currentTimeMillis() - this.driveStartTime, 2) + this.base;
            }*/

            // test
            if (this.slowingDown) {
                value = this.SlowDown();
            }
            else {
                value = this.SpeedUp();
            }

            this.speedMultiplier = value;

            if (this.speedMultiplier > 1) {
                this.speedMultiplier = 1;
            }
            else if (this.speedMultiplier < this.base) {
                this.speedMultiplier = this.base;
            }
        }

        // quadratic formulas for slowing down and speeding up
        private double SlowDown() {
            // define the three points
            final double x1 = -this.millisToSlowDown;
            final double y1 = this.base;
            final double x2 = 0;
            final double y2 = 1;
            final double x3 = this.millisToSlowDown;
            final double y3 = this.base;

            final double elapsedTime = System.currentTimeMillis() - this.slowStartTime;

            // return y
            return ((elapsedTime-x2) * (elapsedTime-x3)) / ((x1-x2) * (x1-x3)) * y1 +
                    ((elapsedTime-x1) * (elapsedTime-x3)) / ((x2-x1) * (x2-x3)) * y2 +
                    ((elapsedTime-x1) * (elapsedTime-x2)) / ((x3-x1) * (x3-x2)) * y3;
        }

        private double SpeedUp() {
            // define the three points
            // constants used for speeding up and slowing down
            final double x1 = -this.millisToSpeedUp;
            final double y1 = 1;
            final double x2 = 0;
            final double y2 = this.base;
            final double x3 = this.millisToSpeedUp;
            final double y3 = 1;

            final double elapsedTime = System.currentTimeMillis() - this.driveStartTime;

            // return y
            return ((elapsedTime-x2) * (elapsedTime-x3)) / ((x1-x2) * (x1-x3)) * y1 +
                    ((elapsedTime-x1) * (elapsedTime-x3)) / ((x2-x1) * (x2-x3)) * y2 +
                    ((elapsedTime-x1) * (elapsedTime-x2)) / ((x3-x1) * (x3-x2)) * y3;
        }
    }


    // called in op mode methods -------------------------------------------------------------------
    // called inside the autonomous program's while loop
    public void HandleInstructions(long elapsedTime, double correction) {
        // handle all sequential instructions
        ArrayList<SequentialInstruction> deadInstructions = new ArrayList<>();

        // sequentially perform the instructions
        if (seqInstructions.size() > 0) {
            if (elapsedTime >= (drivingDelay + seqInstructions.get(0).delay) && !seqInstructions.get(0).started) {
                seqInstructions.get(0).Perform();
                seqInstructions.get(0).started = true;
                if (seqInstructions.get(0).dead) {
                    seqInstructions.remove(0);
                }
            }
            else if (!seqInstructions.get(0).dead && seqInstructions.get(0).started) {
                seqInstructions.get(0).Check(correction);
                if (seqInstructions.get(0).dead) {
                    seqInstructions.remove(0);
                    drivingDelay = elapsedTime;
                }
            }
        }
    }

    // called to auto kill op mode when instructions are done
    public boolean StillRunning() {
        if (this.InstructionLeft() > 0) return true;
        return false;
    }

    // called to get total of instructions left
    public int InstructionLeft() {
        return seqInstructions.size() + timerInstructions.size();
    }

    // adding methods ------------------------------------------------------------------------------
    // methods to easily add instructions to the internal instruction array lists
    public void AddSeqMotorPowerInstruction(long delay, DcMotor motor, double power) {
        seqInstructions.add(new SequentialMotorPowerInstruction(delay, motor, power));
    }

    public void AddSeqMotorDistanceInstruction(long delay, DcMotor motor, double inches, boolean wait) {
        seqInstructions.add(new SequentialMotorDistanceInstruction(delay, motor, inches, wait));
    }

    public void AddSeqServoInstruction(long delay, Servo servo, double position, boolean wait) {
        seqInstructions.add(new SequentialServoInstruction(delay, servo, position, wait));
    }

    public void AddSeqDrivingInstruction(long delay, ArrayList<DcMotor> driveMotors, double inches, int methodKey) {
        seqInstructions.add(new SequentialDrivingInstruction(delay, driveMotors, inches, methodKey));
    }

    public void AddTestDrive(long delay, ArrayList<DcMotor> driveMotors, double inches, int methodKey) {
        seqInstructions.add(new TestMotorDrive(delay, driveMotors, inches, methodKey));
    }
}