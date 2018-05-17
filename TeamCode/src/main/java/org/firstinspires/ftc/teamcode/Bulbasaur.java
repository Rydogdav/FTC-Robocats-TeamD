/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *mn
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class Bulbasaur extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    //public DcMotor motorLeftIntake = null;
    //public DcMotor motorRightIntake = null;
    public DcMotor motorFrontLift = null;
    public Servo servoBackJewel = null;
    public Servo servoGrabRightTop = null;
    public Servo servoGrabRightBottom = null;
    public Servo servoGrabLeftTop = null;
    public Servo servoGrabLeftBottom = null;
    public ColorSensor sensorBackJewel = null;

    public final double SERVO_OPEN = 0.0;
    public final double SERVO_CLOSED = 1.0;
    public int toggle = -1;
    public int x = 0;
    public double servoDegrees;
    public double servoEquation = 1/255 * servoDegrees;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFrontLeft  = hardwareMap.get(DcMotor.class, "left_drive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_drive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "left_omni");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_omni");
        //motorLeftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        //motorRightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        motorFrontLift = hardwareMap.get(DcMotor.class, "front_lift");
        servoBackJewel = hardwareMap.get(Servo.class, "jewel_servo");
        servoGrabRightTop = hardwareMap.get(Servo.class, "right_top");
        servoGrabRightBottom = hardwareMap.get(Servo.class, "right_bottom");
        servoGrabLeftTop = hardwareMap.get(Servo.class, "left_top");
        servoGrabLeftBottom = hardwareMap.get(Servo.class, "left_bottom");
        sensorBackJewel = hardwareMap.get(ColorSensor.class, "jewel_sensor");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        //motorLeftIntake.setDirection(DcMotor.Direction.FORWARD);
        //motorRightIntake.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLift.setDirection(DcMotor.Direction.FORWARD);

        servoBackJewel.setPosition(200.0/256);

        double intakePower = 0;
        waitForStart();
        runtime.reset();


        /*servoGrabLeftTop.setPosition(1.0);
        telemetry.addLine("1.0");
        telemetry.update();
        sleep(3000);
        servoGrabLeftTop.setPosition(0.0);
        telemetry.addLine("0.0");
        telemetry.update();*/

        double leftPower;
        double rightPower;
        double liftPower = 0;
        double gearMult = 1;
        double closePositonTop = 0.35;      // smaller the value makes the arms close more
        double closePostionBottom = 0.30;
        while (opModeIsActive()) {

            leftPower = -gamepad1.left_stick_y * gearMult;
            rightPower = -gamepad1.right_stick_y * gearMult;
            if(gamepad1.right_bumper){   //185 145 25 60
                servoGrabRightTop.setPosition(1);
                servoGrabLeftTop.setPosition(0);
                servoGrabRightBottom.setPosition(1);
                servoGrabLeftBottom.setPosition(0);
            }
            if(gamepad1.left_bumper){
                servoGrabRightTop.setPosition(0+closePositonTop);
                servoGrabLeftTop.setPosition(1-closePositonTop);
                servoGrabRightBottom.setPosition(0+closePostionBottom);
                servoGrabLeftBottom.setPosition(1-closePostionBottom);
            }
            if (gamepad1.y) gearMult = 1;

            if (gamepad1.a) gearMult = 0.5;

            if (gamepad1.left_trigger > 0) liftPower = 1;
            else if (gamepad1.right_trigger > 0) liftPower = -1;
            else liftPower = 0;



            /*if ((gamepad1.left_trigger > 0.8)) {
                telemetry.addLine("left trigger pressed");
                intakePower = -.6;
            } else if ((gamepad1.left_trigger > .2) && (intakePower == -.6))
                intakePower = 0;

            //if (gamepad1.right_trigger > 0.5)intakePower = 1;
            if (gamepad1.right_trigger > .8) {
                telemetry.addLine("right trigger pressed");
                telemetry.update();
                if (intakePower == 0)
                    intakePower = 1;
                else if (intakePower == 1)
                    intakePower = 0;
                //sleep(500);
                while (gamepad1.right_trigger > .2) {
                    idle();
                }
            }*/


            // Send calculated power to wheels
            motorFrontLeft.setPower(leftPower);
            motorBackLeft.setPower(leftPower);
            motorFrontRight.setPower(rightPower);
            motorBackRight.setPower(rightPower);
            motorFrontLift.setPower(liftPower);
            /*motorLeftIntake.setPower(intakePower);
            motorRightIntake.setPower(intakePower);*/

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            idle();
        }
    }
}

