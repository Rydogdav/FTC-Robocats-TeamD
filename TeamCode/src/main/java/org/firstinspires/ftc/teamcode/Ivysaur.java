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
    public ColorSensor colorSensor = null;
    public double servoDegrees;
    public double servoEquation = 1 / 255 * servoDegrees;

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_MILLIMETERS = 100.0;     // For figuring circumference
    static final double COUNTS_PER_MILLIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MILLIMETERS * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double sensordown = 20;
    static final double sensorup = 200;
    static final int colorblue = -1;
    static final int colorred = 1;
    static final int ballred = colorred;
    static final int ballblue = colorblue;
    static final double jewelknockdistance = 70;
    static final double zonedistance = 700;
    static final double finalpushdist = 50;
    static final int clockwise = 1;
    static final int anticlockwise = -1;
    static final int glyphside = 1;
    static final int relicside = -1;
    static int ballcolor = colorred;
    static int rotation = 0;



    static int startside = relicside;
    static int teamcolor = colorblue;
    static boolean confirm = false;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFrontLeft  = hardwareMap.get(DcMotor.class, "left_omni");
        motorFrontRight = hardwareMap.get(DcMotor.class, "right_omni");
        motorBackLeft   = hardwareMap.get(DcMotor.class, "left_drive");
        motorBackRight  = hardwareMap.get(DcMotor.class, "right_drive");

        servoBackJewel = hardwareMap.get(Servo.class, "jewel_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "jewel_sensor");
        colorSensor.enableLed(true);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        moveSensor(sensorup);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        //----------Some Ryan Code----------
        setParam();

        waitForStart();
        runtime.reset();
        telemetry.addLine("Starting auto");
        telemetry.update();
        doAutonomous();
        telemetry.addLine("Done auto");
        telemetry.update();

        
    }

    public void DoTest()
    {
        /*
        motorFrontLeft.setPower(.4);
        sleep(4000);
        motorFrontLeft.setPower(0);

        motorBackLeft.setPower(.4);
        sleep(4000);
        motorBackLeft.setPower(0);

        motorFrontRight.setPower(.4);
        sleep(4000);
        motorFrontRight.setPower(0);

        motorBackRight.setPower(.4);
        sleep(4000);
        motorBackRight.setPower(0);
        */

        driveStraight(.4,311, 6);

    }

    public void autotesting (double position) {
        servoBackJewel.setPosition(position /256);


    }



    public void setParam () {
      /*  while(!confirm && opModeIsActive()){
            telemetry.addData("Team Color", teamcolor);
            telemetry.addData("Robot Placement", startside);
            telemetry.addLine("Press A for Red Alliance, B for Blue Alliance.");
            telemetry.addLine("Press X for Glyph Side placement, Y for Relic Side.");
            telemetry.addLine("Press both bumpers to confirm.");
            //Not set in stone, just conceptual
            telemetry.update();
            if (gamepad1.a) {
                teamcolor = colorred;
            }
            if (gamepad1.b) {
                teamcolor = colorblue;
            }
            if (gamepad1.x) {
                startside = glyphside;
            }
            if (gamepad1.y) {
                startside = relicside;
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper){
                confirm = true;
            }
        }
   */ }

    public void doAutonomous(){
        telemetry.addLine("Sensor down");
        telemetry.update();

        moveSensor(sensordown);
        sleep(2000);

        telemetry.addLine("Detecting color");
        telemetry.update();
        detectcolor();

        telemetry.addLine("Knocking jewel");
        telemetry.update();
        knockjeweloff();
        sleep(2000);
        telemetry.addLine("Sensor up");
        telemetry.update();
        moveSensor(sensorup);
        sleep(2000);

        if ((teamcolor == colorblue) && (startside == glyphside))
            rotation = clockwise;
        else if ((teamcolor == colorblue) && (startside == relicside))
            rotation = anticlockwise;
        else if ((teamcolor == colorred) && (startside == glyphside))
            rotation = anticlockwise;
        else if ((teamcolor == colorred) && (startside == relicside))
            rotation = clockwise;
        parkInZone(-teamcolor, rotation);
        Turn90(1);
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
            telemetry.addLine("Ball Color N/A");
            telemetry.update();
        }
    }

    public void Turn90(double turn) {
        encoderDrive(.5, -127 * turn, 127 * turn, 6);       //(Left Wheel Distance (IN.), Right-Wheel Distance, Timeout (Sec))
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
            correctiondistance = jewelknockdistance;
        else if (ballcolor != 0)
            correctiondistance = -jewelknockdistance;
        else
            correctiondistance = 0;
        driveStraight(.4, (zonedistance + correctiondistance)*direction, 6);
       // turnRobot(rotation, 45);
       // driveStraight(.4, finalpushdist*direction, 6);
    }
    public void turnRobot(int rotationDirection, int angle){
        double distance;
        distance=angle*235/90;
        encoderDrive(.4, rotationDirection*distance, -rotationDirection*distance, 6);

    }

    public void driveStraight(double speed, double distanceinmilimeters, double timeoutS){

        encoderDriveV1(speed, distanceinmilimeters, distanceinmilimeters, timeoutS);
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

        runtime.reset();
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            telemetry.addData("Path1", "Running to %7d :%7d", targetLeft, targetRight);
           // telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d", FLPos, BLPos, FRPos, BRPos);
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
        int newLeftTarget;
        int newRightTarget;
        int leftDirection;
        int rightDirection;
        if (leftMillimeters >= 0)
            leftDirection = 1;
        else leftDirection = -1;
        if (rightMillimeters >= 0)
            rightDirection = 1;
        else rightDirection = -1;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
//            newLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (leftMillimeters * COUNTS_PER_MILLIMETERS);
//            newRightTarget = motorFrontRight.getCurrentPosition() + (int) (rightMillimeters * COUNTS_PER_MILLIMETERS);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            newLeftTarget = (int)( leftMillimeters * COUNTS_PER_MILLIMETERS * leftDirection);
            newRightTarget = (int)( rightMillimeters * COUNTS_PER_MILLIMETERS* rightDirection);

            motorFrontLeft.setTargetPosition((int) newLeftTarget);
            motorFrontRight.setTargetPosition((int) newRightTarget);
            motorBackLeft.setTargetPosition((int) newLeftTarget);
            motorBackRight.setTargetPosition((int) newRightTarget);

            // reset the timeout time and start motion.00.
            motorFrontLeft.setTargetPosition((int) (leftMillimeters * COUNTS_PER_MILLIMETERS));
            motorFrontRight.setTargetPosition((int) (rightMillimeters * COUNTS_PER_MILLIMETERS));
            motorBackLeft.setTargetPosition((int) (leftMillimeters * COUNTS_PER_MILLIMETERS));
            motorBackRight.setTargetPosition((int) (rightMillimeters * COUNTS_PER_MILLIMETERS));


            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition(),
                        motorBackLeft.getCurrentPosition(),
                        motorBackRight.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}

