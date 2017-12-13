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

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name="autonomous red1", group="Linear Opmode")

public class Ivysaur extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor  motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public Servo servoBackJewel = null;
    //public ColorSensor sensorBackJewel = null;
    public ColorSensor colorSensor = null;
    public double servoDegrees;
    public double servoEquation = 1 / 255 * servoDegrees;

    static final double COUNTS_PER_MOTOR_REV = 4;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_MILLIMETERS = 94.0;     // For figuring circumference
    static final double COUNTS_PER_MILLIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MILLIMETERS * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double sensordown = 40;
    static final double sensorup = 200;
    static final int colorblue = -1;
    static final int colorred = 1;
    static final int ballred = colorred;
    static final int ballblue = colorblue;
    static final double jewelknockdistance = 70;
    static final double zonedistance = 300;
    static final double finalpushdist = 50;
    static final int clockwise = 1;
    static final int anticlockwise = -1;
    static final int glyphside = 1;
    static final int relicside = -1;
    static int ballcolor = colorred;
    static int rotation = 0;



    static int startside = glyphside;
    static int teamcolor = colorblue;
    static boolean confirm = false;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFrontRight = hardwareMap.get(DcMotor.class, "right_drive");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "left_drive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "left_omni");
        motorBackRight = hardwareMap.get(DcMotor.class, "right_omni");

        servoBackJewel = hardwareMap.get(Servo.class, "jewel_servo");
        //sensorBackJewel = hardwareMap.get(ColorSensor.class, "jewel_sensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "jewel_sensor");
        colorSensor.enableLed(true);

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        moveSensor(sensorup);

        //----------Some Ryan Code----------
        //setParam();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        telemetry.addLine("Starting auto");
        telemetry.update();
        //doAutonomous();
        driveStraight(.4,600, 6);
        sleep(5000);
        telemetry.addLine("Done auto");
        telemetry.update();

        
    }
    public void setParam () {
        while(!confirm && opModeIsActive()){
            telemetry.addData("Team Color", teamcolor);
            telemetry.addData("Robot Placement", startside);
            telemetry.addLine("Press A for Red Alliance, B for Blue Alliance.");
            telemetry.addLine("Press X for Glyph Side placement, Y for Relic Side.");
            telemetry.addLine("Press both bumpers to confirm.");
            //Not set in stone, just conceptual
            telemetry.update();
            if (gamepad1.a) {
                teamcolor = colorred;
                telemetry.addLine("RED TEAM SELECTED");
                telemetry.update();
            }
            if (gamepad1.b) {
                teamcolor = colorblue;
                telemetry.addLine("BLUE TEAM SELECTED");
                telemetry.update();
            }
            if (gamepad1.x) {
                startside = glyphside;
                telemetry.addLine("START @ GLYPH");
                telemetry.update();
            }
            if (gamepad1.y) {
                startside = relicside;
                telemetry.addLine("START @ RELIC");
                telemetry.update();
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper){
                confirm = true;
            }
        }
    }

    public void doAutonomous(){
        telemetry.addLine("Sensor down");
        telemetry.update();

        moveSensor(sensordown);
        sleep(1000);

        telemetry.addLine("Detecting color");
        telemetry.update();
        detectcolor();

        telemetry.addLine("Knocking jewel");
        telemetry.update();
        knockjeweloff();
        sleep(1000);
        telemetry.addLine("Sensor up");
        telemetry.update();
        moveSensor(sensorup);
        sleep(1000);

        if ((teamcolor == colorblue) && (startside == glyphside))
            rotation = clockwise;
        else if ((teamcolor == colorblue) && (startside == relicside))
            rotation = anticlockwise;
        else if ((teamcolor == colorred) && (startside == glyphside))
            rotation = anticlockwise;
        else if ((teamcolor == colorred) && (startside == relicside))
            rotation = clockwise;

        parkInZone(-teamcolor, rotation);
    }

    public void moveSensor(double position){ //sensor moves
        servoBackJewel.setPosition(position /256);

    }
    public void detectcolor() {

        //detect ball color
         if (colorSensor.blue() > colorSensor.red()){
            ballcolor = ballblue;
            telemetry.addLine("Ball Color Blue");
            telemetry.update();
        }
         else if  (colorSensor.blue() < colorSensor.red()) {
            ballcolor = ballred;
            telemetry.addLine("Ball Color Red");
            telemetry.update();
        }
        else{
            ballcolor = 0;
            telemetry.addLine("Ball Color Unknown");
            telemetry.update();
        }
    }

    public void knockjeweloff(){
        if (ballcolor == teamcolor) {
            driveStraight(.4, -jewelknockdistance, 6);
        }
        else if (ballcolor != 0){
            driveStraight(.4, jewelknockdistance, 6);
        }
    }

    public void parkInZone(int direction, int rotation) {
        double correctiondistance;
        if (ballcolor == teamcolor)
            correctiondistance = -jewelknockdistance;
        else if (ballcolor != 0)
            correctiondistance = jewelknockdistance;
        else
            correctiondistance = 0;
        driveStraight(.4, (zonedistance + correctiondistance)*direction, 6);
        turnRobot(rotation, 45);
        driveStraight(.4, finalpushdist*direction, 6);
    }
    public void turnRobot(int rotationDirection, int angle){
        double distance;
        distance=angle*235/90;
        encoderDrive(.4, rotationDirection*distance, -rotationDirection*distance, 6);

    }

    public void driveStraight(double speed, double distanceinmilimeters, double timeoutS){

        encoderDrive(speed, distanceinmilimeters, distanceinmilimeters, timeoutS);
    }

    public void encoderDrive(double speed,
                             double leftMillimeters, double rightMillimeters,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int leftDirection;
        int rightDirection;
        int Running = 1;
        int BLPos;
        int FLPos;
        int BRPos;
        int FRPos;
        double leftPower;
        double rightPower;
        double targetLeft;
        double targetRight;

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (leftMillimeters >= 0) {
            leftDirection = 1;
            leftPower = speed;
        }
        else {
            leftDirection = -1;
            leftPower = -speed;
        }

        if (rightMillimeters >= 0) {
            rightDirection = 1;
            rightPower = speed;
        }
        else {
            rightDirection = -1;
            rightPower = -speed;
        }

        targetLeft = Math.abs(leftMillimeters);
        targetRight = Math.abs(rightMillimeters);

        runtime.reset();
        while ((Running == 1) && opModeIsActive() && (runtime.seconds() < timeoutS))
        {
            BLPos = Math.abs(motorBackLeft.getCurrentPosition());
            FLPos = Math.abs(motorFrontLeft.getCurrentPosition());
            BRPos = Math.abs(motorBackLeft.getCurrentPosition());
            FRPos = Math.abs(motorFrontLeft.getCurrentPosition());

            if (BLPos > targetLeft)
                leftPower = 0;

            if (FLPos > targetLeft)
                leftPower = 0;

            if (BRPos > targetRight)
                rightPower = 0;

            if (FRPos > targetRight)
                rightPower = 0;

            if ((leftPower == 0) && (rightPower == 0))
                Running = 0;

            motorFrontLeft.setPower(leftPower);
            motorBackLeft.setPower(leftPower);
            motorFrontRight.setPower(rightPower);
            motorBackRight.setPower(rightPower);

            telemetry.addData("Path1", "Running to %7d :%7d", (int)targetLeft, (int)targetRight);
            telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d", FLPos, BLPos, FRPos, BRPos);
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void encoderDriveV1(double speed,
                             double leftMillimeters, double rightMillimeters,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget  = motorFrontLeft.getCurrentPosition()  - (int)( leftMillimeters * COUNTS_PER_MILLIMETERS);
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int)( rightMillimeters * COUNTS_PER_MILLIMETERS);
            newBackLeftTarget   = motorBackLeft.getCurrentPosition()   + (int)( leftMillimeters * COUNTS_PER_MILLIMETERS);
            newBackRightTarget  = motorBackRight.getCurrentPosition()  + (int)( rightMillimeters * COUNTS_PER_MILLIMETERS);


            motorFrontLeft.setTargetPosition((int) newFrontLeftTarget);
            motorFrontRight.setTargetPosition((int) newFrontRightTarget);
            motorBackLeft.setTargetPosition((int) newBackLeftTarget);
            motorBackRight.setTargetPosition((int) newBackRightTarget);


            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
//            motorFrontRight.setPower(Math.abs(speed));
//            motorBackLeft.setPower(Math.abs(speed));
//            motorBackRight.setPower(Math.abs(speed));

            telemetry.addLine("Drive start");
            telemetry.update();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy()) &&
                    (motorBackLeft.isBusy() && motorBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition(),
                        motorBackLeft.getCurrentPosition(),
                        motorBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}

