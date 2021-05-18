package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="LearningDriveCode", group="Test")
public class LearningDriveCode extends OpMode{

    // DEFINE robot
    RobotHardware robot = new RobotHardware();
    long startTime;

    public void GoForward() {
        robot.motor1.setPower(1);
        robot.motor2.setPower(1);
        robot.motor3.setPower(1);
        robot.motor4.setPower(1);


    }

    public void GoBack(){
        robot.motor1.setPower(-1);
        robot.motor2.setPower(-1);
        robot.motor3.setPower(-1);
        robot.motor4.setPower(-1);
    }

    public void TurnLeft(){
        robot.motor1.setPower(-1);
        robot.motor2.setPower(1);
        robot.motor3.setPower(-1);
        robot.motor4.setPower(1);
}
    public void TurnRight(){
        robot.motor1.setPower(1);
        robot.motor2.setPower(-1);
        robot.motor3.setPower(1);
        robot.motor4.setPower(-1);
    }

    public void StrafeLeft(){
        robot.motor1.setPower(-1);
        robot.motor2.setPower(1);
        robot.motor3.setPower(1);
        robot.motor4.setPower(-1);
    }
    public void StrafeRight(){
        robot.motor1.setPower(1);
        robot.motor2.setPower(-1);
        robot.motor3.setPower(-1);
        robot.motor4.setPower(1);
    }
    public void ShootyShootyBangBang(){
        robot.motor7.setPower(1);
    }
    public void TacticalReload(){
        robot.servo1.setPosition(1);
    }
    public void TacticalReloadTheReload() {
        robot.servo1.setPosition(0);
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
    // RUN ONCE ON init()
    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    //LOOP ON init()
    // reminder dylan sucks
    @Override
    public void init_loop() {

    }

    //RUN ONCE ON start()
    @Override
    public void start() {
        startTime = System.currentTimeMillis();

    }

    //LOOP ON start()
    @Override
    public void loop() {
        long elapsedTime = TimeElapsed();

        if (elapsedTime < 2000) {
            StrafeRight();
        }

        else if (elapsedTime < 4000) {
            GoForward();
        }
        else if (elapsedTime < 4500){
            Kill();
        }


        else if (elapsedTime < 5500) {
            ShootyShootyBangBang();
        }
        else if (elapsedTime < 7500) {
            TacticalReload();
        }
        else if (elapsedTime < 8500) {
            TacticalReloadTheReload();
        }

        else {

            Kill();

        }

        telemetry.addData("Elapsed Time: ", elapsedTime);
    }

    // RUN ONCE ON stop()
    @Override
    public void stop() {

    }

}