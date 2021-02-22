package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

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
    private long drivingDelay = 0;

    // these keys make typing them into the add instruction methods easier to autocomplete
    public final int driveForward = 0;
    public final int driveBackward = 1;
    public final int strafeLeft = 2;
    public final int strafeRight = 3;
    public final int turnLeft = 4;
    public final int turnRight = 5;

    // instruction list
    ArrayList<SequentialInstruction> seqInstructions = new ArrayList<>();
    ArrayList<TimerInstruction> timerInstructions = new ArrayList<>();

    // base instructions ---------------------------------------------------------------------------
    // base instruction class for sequential instructions
    private class SequentialInstruction {
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
    private class TimerInstruction {
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
        private double speedMultiplier = 0;
        private long driveStartTime = 0;

        /*
        initialization constructor
         */
        public SequentialDrivingInstruction(long delay, ArrayList<DcMotor> driveMotors, double inches, int methodKey) {
            super(delay);
            this.driveMotors = driveMotors;
            this.distance = (int)Math.round(inches * TICKS_PER_INCHES);
            this.method = methodKey;
            this.drive = true;
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

            // set the motor speed
            for (DcMotor motor: this.driveMotors) {
                motor.setPower(speed);
            }

            // set the drive start time
            this.driveStartTime = System.currentTimeMillis();
        }

        /*
        check method
            uses imu to correct if not turning, if turning make sure turning correctly
         */
        public void Check(double correction) {
            if (this.method == turnLeft || this.method == turnRight) { // turning

            }
            else {
                // check if dead first
                for (DcMotor motor: this.driveMotors) { // not turning
                    if (motor.isBusy()) {
                        this.dead = false;
                    }
                }

                // apply correction otherwise
                if (!this.dead) {
                    // adjust the multiplier
                    this.AdjustMultiplier();

                   // add the correction correctly
                    this.driveMotors.get(0).setPower((speed - correction) * this.speedMultiplier);
                    this.driveMotors.get(1).setPower((speed + correction) * this.speedMultiplier);
                    this.driveMotors.get(2).setPower((speed - correction) * this.speedMultiplier);
                    this.driveMotors.get(3).setPower((speed + correction) * this.speedMultiplier);
                }
            }
        }

        // speed multiplier function
        private void AdjustMultiplier() {
            double quadraticConstantA = (3 * Math.pow(10, -8));
            this.speedMultiplier = quadraticConstantA * Math.pow(System.currentTimeMillis() - this.driveStartTime, 2) + .25; // exponential function
            if (this.speedMultiplier > 1) {
                this.speedMultiplier = 1;
            }
        }
    }

    // timer instructions --------------------------------------------------------------------------


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

}
