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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="autonomous red", group="Linear Opmode")

public class Ivysaur extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;

    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public DcMotor leftOmni = null;
    public DcMotor rightOmni = null;
    public Servo jewelServo = null;
    public ColorSensor jewelSensor = null;
    public double  servoDegrees;
    public double servoEquation = 1/255 * servoDegrees;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motorFrontLeft  = hardwareMap.get(DcMotor.class, "right_drive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "left_drive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "left_omni");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_omni");

        motorFrontLeft  = hardwareMap.get(DcMotor.class, "left_drive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_drive");
        leftOmni = hardwareMap.get(DcMotor.class, "left_omni");
        rightOmni = hardwareMap.get(DcMotor.class, "right_omni");

        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        jewelSensor = hardwareMap.get(ColorSensor.class, "jewel_sensor");
        jewelSensor.enableLed(true);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        leftOmni.setDirection(DcMotor.Direction.FORWARD);
        rightOmni.setDirection(DcMotor.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        motorFrontLeft.setPower(-.35);

        motorBackLeft.setPower(-.35);
        motorFrontRight.setPower(-.35);
        motorBackRight.setPower(-.35);
        sleep(1500);                             // This was changed from 2000. then 500
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
/**
        leftOmni.setPower(-.35);
        motorFrontRight.setPower(-.35);
        rightOmni.setPower(-.35);
        sleep(2000);
        motorFrontLeft.setPower(0);
        leftOmni.setPower(0);
        rightOmni.setPower(0);

        motorFrontRight.setPower(0);
        jewelServo.setPosition(165);
        if (jewelSensor.blue(true) > jewelSensor.red(false)) {
            motorFrontRight.setPower(.35);

            motorBackRight.setPower(.35);
            motorFrontLeft.setPower(-.35);
            motorBackLeft.setPower(-.35);
            sleep(1000);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            rightOmni.setPower(.35);
            motorFrontLeft.setPower(-.35);
            leftOmni.setPower(-.35);
            sleep(1000);
            motorFrontLeft.setPower(0);
            leftOmni.setPower(0);
            rightOmni.setPower(0);

            motorFrontRight.setPower(0);
        }
        if (jewelSensor.blue(false) < jewelSensor.red(true)) {
            motorFrontRight.setPower(-.35);

            motorBackRight.setPower(-.35);
            motorFrontLeft.setPower(.35);
            motorBackLeft.setPower(.35);
            sleep(1000);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            rightOmni.setPower(-.35);
            motorFrontLeft.setPower(.35);
            leftOmni.setPower(.35);
            sleep(1000);
            motorFrontLeft.setPower(0);
            leftOmni.setPower(0);
            rightOmni.setPower(0);

            motorFrontRight.setPower(0);
        } */






        }
    }

