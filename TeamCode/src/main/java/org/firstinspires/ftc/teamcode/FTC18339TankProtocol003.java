package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FTC18339TankProtocol003 extends Main {
    private double left_front_power = 0;
    private double left_back_power = 0;
    private double right_front_power = 0;
    private double right_back_power = 0;

    private double moveMultiplier = 1f;

    int dir = 1;
    double adder = 0;

    public double[] currentCraneCoordinates = new double[] {0,0};

    public double[] groundPosition = new double[] {207.24, 164.24};

    @Override
    public void runOpMode() {

        //Initialize all hardware, servos, and sensors, as well as the algorithm
        initMaths();
        initHardware();
        initManualModes();

        //wait until the play button in pressed
        waitForStart();

        //Initial Position
        double[] qs = math.IKArm(7 * Algorithms002.mmPerInch, 7 * Algorithms002.mmPerInch, Math.PI);

        if(qs == null || Double.isNaN(qs[0]) || Double.isNaN(qs[1]) || Double.isNaN(qs[2])) {
        } else {
            base_arm_joint.setPosition(math.Clamp(qs[0] / (3 * Math.PI / 2), 0, 1));
            second_arm_joint.setPosition(math.Clamp(qs[1], 0, 1));
            hand.setPosition(math.Clamp(qs[2], 0, 1));
            gripper1.setPosition(1);

            currentX = 7 * Algorithms002.mmPerInch;
            currentY = 7 * Algorithms002.mmPerInch;
            currentPhi = Math.PI;
        }

        //Is the program started?
        if (opModeIsActive()) {
            //While the program is started.
            while (opModeIsActive()) {
                // Show motor info on android phone

                //Runtime Loop to update hardware with input, and automation
                SetMotorForces();
                SetSpinnerForces();
                SetCraneIKTarget();
                SetGripperForces();

                //telemetry.addData("q1; q2; q3", Math.toDegrees(math.tQ1) + "; " + Math.toDegrees(math.tQ2) + "; " + Math.toDegrees(math.tQ3));

                //go to the ground position
                if(gamepad2.x) {
                    double[] qg = math.IKArm(groundPosition[0], -groundPosition[1], Math.PI / 2);

                    if(qg == null || Double.isNaN(qg[0]) || Double.isNaN(qg[1]) || Double.isNaN(qg[2])) {
                    } else {
                        base_arm_joint.setPosition(math.Clamp(1 - (qg[0] / (Math.PI)), 0, 1));
                        second_arm_joint.setPosition(math.Clamp(1 - (qg[1] / (3 * Math.PI / 2)), 0, 1));
                        hand.setPosition(math.Clamp(qg[2] / (3 * Math.PI / 2), 0, 1));

                        currentX = -groundPosition[0];
                        currentY = -groundPosition[1];
                        currentPhi = Math.PI / 2;
                    }
                }

                if(gamepad2.y) {
                    double[] qr = math.IKArm(math.startPosition.x, math.startPosition.y, math.startPosition.z);

                    if(qr == null || Double.isNaN(qr[0]) || Double.isNaN(qr[1]) || Double.isNaN(qr[2])) {
                    } else {
                        base_arm_joint.setPosition(math.Clamp(1 - (qr[0] / Math.PI), 0, 1));
                        second_arm_joint.setPosition(math.Clamp(1 - (qr[1] / (3 * Math.PI / 2)), 0, 1));
                        hand.setPosition(math.Clamp(1 - (qr[2] / (3 * Math.PI / 2)), 0, 1));

                        currentX = math.startPosition.x;
                        currentY = math.startPosition.y;
                        currentPhi = math.startPosition.z;
                    }
                }

                //Update phone info
                telemetry.update();
                idle();
            }
        }
    }

    boolean wasMovingLastFrame = false;
    boolean notRunningConveyor = true;
    boolean notRunningRoller = true;
    public void SetMotorForces() {
        if(!NoNullHardware()) return;

        //Algorithm determined wheel forces with the inputs
        double y = gamepad1.left_stick_y;

        //Right trigger right rotation
        //Right trigger is right rotation, left trigger is left. Can slow rotations by depressing trigger less.
        float rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        //find the speeds of each wheel
        left_front_power = math.GetWheelForceTank(y, 1, rAxis);
        left_back_power = math.GetWheelForceTank(y, 2, rAxis);
        right_front_power = math.GetWheelForceTank(y, 3, rAxis);
        right_back_power = math.GetWheelForceTank(y, 4, rAxis);

        //Show the powers on the telemetry screen
        telemetry.addData("lpb", left_back_power);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        //Set the velocities of each wheel motor
        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
    }

    public void SetSpinnerForces() {
        if(!NoNullSpinner()) return;

        //Get the inputs
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;

        //Set the spinner forces
        if(x) {
            telemetry.addData("x","x is active");
            spinner.setPower(-1);
        } else if(b) {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }
    }

    float craneRotationSpeedMultiplier = 0.002f;
    float craneArmJointMultiplier = 1.5f;
    float craneArmOrientationMultiplier = 0.02f;
    double currentX = 8 * Algorithms002.mmPerInch;
    double currentY = 8 * Algorithms002.mmPerInch;
    double currentPhi = Math.PI / 2;

    public void SetCraneIKTarget()
    {
        //placeholder variables for target
        double phi = currentPhi;

        double xChange = 0, yChange = 0, phiChange = 0;

        double orientationChange = 0;

        telemetry.addData("gamepad2left; gamepad2right", gamepad2.left_stick_y + "; " + gamepad2.right_stick_y);

        if(gamepad2.left_bumper)
            orientationChange = -1;
        else if(gamepad2.right_bumper)
            orientationChange = 1;

        //Source the ticks in the rotator
        float ticks = MAX_NUM_TICKS_ROTATOR;
        //what swath of the circle can the rotator rotate to? Half of the actual angle so 45 degrees represents a quarter of the circle
        float angleOfRotation = 45;
        //Half of the total range, positive and negative angle of rotation
        float rangeOfTicks = (angleOfRotation/360) * ticks;
        //Set the velocity
        double r = (gamepad2.right_trigger - gamepad2.left_trigger) * craneRotationSpeedMultiplier * ROTATOR_RPM * ticks;
        arm_rotator.setVelocity(r);
        //if(!(arm_rotator.getCurrentPosition() >= rangeOfTicks && r > 0) && !(arm_rotator.getCurrentPosition() <= -rangeOfTicks && r < 0)) {

        //}

        if((gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0 && orientationChange == 0)) return;

        //Get the inputs for the arm
        xChange = -gamepad2.left_stick_y * craneArmJointMultiplier;
        yChange = -gamepad2.right_stick_y * craneArmJointMultiplier;
        phiChange = orientationChange * craneArmOrientationMultiplier;

        //Clamp the controls
        double[] target = math.IKTargetClamp(currentX + xChange, currentY + yChange);
        phi += phiChange;

        //Calculate the angles the arm must take
        double[] qs = math.IKArm(target[0], target[1], phi);

        if(qs == null || Double.isNaN(qs[0]) || Double.isNaN(qs[1]) || Double.isNaN(qs[2]))
            return;

        telemetry.addData("fQ1; fQ2; fQ3", Math.toDegrees(qs[0]) + "; " + Math.toDegrees(qs[1]) + "; " + Math.toDegrees(qs[2]));

        //Set the Servo Positions
        base_arm_joint.setPosition(math.Clamp(qs[0] / (3 * Math.PI / 2), 0, 1));
        second_arm_joint.setPosition(math.Clamp(qs[1], 0, 1));
        hand.setPosition(math.Clamp(qs[2], 0, 1));

        currentX = target[0];
        currentY = target[1];
        currentPhi = phi;
    }

    float gripperPosition = 1;
    public void SetGripperForces() {
        //close the gripper
        if(gamepad2.a) {
            gripperPosition = 1;
        }

        //open the gripper
        if(gamepad2.b) {
            gripperPosition = 0;
        }

        gripper1.setPosition(gripperPosition);
    }
}
