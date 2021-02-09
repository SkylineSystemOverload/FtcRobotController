package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DriverControlled", group="Test")

public class DriverControlled extends OpMode{

    // DEFINE robot
    RobotHardware robot = new RobotHardware();

    // CONSTANTS
    double G1rightStickY = -gamepad1.right_stick_y;
    double G1leftStickY = -gamepad1.left_stick_y;
    double G1rightStickX = gamepad1.right_stick_x;
    double G1leftStickX = gamepad1.left_stick_x;
    boolean G1a = gamepad1.a;
    boolean G1y = gamepad1.y;
    boolean G1x = gamepad1.x;
    boolean G1b = gamepad1.b;
    double G1LT = gamepad1.left_trigger;
    double G1RT = gamepad1.right_trigger;
    boolean G1dpad_right = gamepad1.dpad_right;
    boolean G1dpad_left = gamepad1.dpad_left;
    boolean G1dpad_up = gamepad1.dpad_up;
    boolean G1dpad_down = gamepad1.dpad_down;
    boolean G1rightBumper = gamepad1.right_bumper;
    boolean G1leftBumper = gamepad1.left_bumper;

    // RUN ONCE ON init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("STATUS", "Initialized");
    }

    //LOOP ON init()
    @Override
    public void init_loop() {
    }

    //RUN ONCE ON start()
    @Override
    public void start() {
    }

    //LOOP ON start()
    @Override
    public void loop() {

        //mecanum drive
        robot.motor1.setPower((G1leftStickY) + (G1leftStickX) + (G1rightStickX));
        robot.motor3.setPower((G1leftStickY) - (G1leftStickX) + (G1rightStickX));
        robot.motor2.setPower((G1leftStickY) - (G1leftStickX) - (G1rightStickX));
        robot.motor4.setPower((G1leftStickY) + (G1leftStickX) - (G1rightStickX));

        // intake statements
        if(G1LT>0) { // run intake
            robot.motor5.setPower(1);
        }
        else if(G1y) { // reverse intake
            robot.motor5.setPower(-1);
        }
        else { // stop intake
            robot.motor5.setPower(0);
        }

        // launcher statements
        if(G1RT>0) { // rev launcher
            robot.motor7.setPower(.7);
        }
        else if(G1rightBumper) { // slower launcher
            robot.motor7.setPower(.55);
        }
        else { // stop launcher
            robot.motor7.setPower(0);
        }

        // shooter and intake servo statements
        if(G1x) { // use shooting and intake clean up servo
            robot.servo1.setPosition(1.2);
            robot.servo3.setPosition(1.2);
        }
        else { // reset shooting and intake clean up servo
            robot.servo1.setPosition(0.5);
            robot.servo3.setPosition(0.275);
        }

        // wobble arm statements
        if(G1a) { // lift wobble arm
            robot.motor6.setPower(.5);
        }
        else if (G1b) { // lower wobble arm
            robot.motor6.setPower(-.5);
        }
        else { // stop wobble arm
            robot.motor6.setPower(0);
        }

        telemetry.addData("motor1 Power", robot.motor1.getPower());
        telemetry.addData("motor2 Power", robot.motor2.getPower());
        telemetry.addData("motor3 Power", robot.motor3.getPower());
        telemetry.addData("motor4 Power", robot.motor4.getPower());
        telemetry.addData("motor5 Power", robot.motor5.getPower());
        telemetry.addData("motor7 Power", robot.motor7.getPower());
    }

    // RUN ONCE ON stop()
    @Override
    public void stop() {
    }
}