package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
/*import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;*/

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Algorithms002 {

    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = 6 * mmPerInch;

    public static final float halfField = 72 * mmPerInch;
    public static final float quadField = 36 * mmPerInch;

    //public static final float lengthOfFirstArmJoint = 5.567f * mmPerInch;
    //public static final float lengthOfSecondArmJoint = 7.630f * mmPerInch;
    public static final float lengthOfFirstArmJoint = 104.1f;
    public static final float lengthOfSecondArmJoint = 193.4f;

    public static final double initialQ1 = 0;
    public static final double initialQ2 = 0;
    public static final double initialQ3 = 0;

    public double currentQ1;
    public double currentQ2;
    public double currentQ3;

    //Really Slow 0.05f, Regular 0.1f
    float controlMultiplier = 0.1f;

    public static final float wheelCircumferenceMm = 301.59f;
    public static final float rotationPerOneRevolution = 2.314f;

    public void Initialize() {
        currentQ1 = initialQ1;
        currentQ2 = initialQ2;
        currentQ3 = initialQ3;
    }

    /*
    The Wheel force is determined for each wheel seperately, but for multidimensional driving, only
    two algorithms are needed. We use the theta determined from the Theta(a,b,c) function and the
    GetQuad(x,y) function. We then check the value of theta and assign different forces for different values.
    */


    //Manipulate values in here in force matrix
    public double GetWheelForce(double x, double y, int i, float x2, double theta, double z)
    {
        if(x == 0 && y == 0 && x2 == 0) {
            return 0;
        }

        double movePower = 0;


        //FORCE MATRIX i == 1 is left_front i == 2 is right_front i == 3 is left_back i == 4 is right_back
        //Uses unit circle math look it up if you need to
        //theta is just divided by pi so 2 is equal in angle to 2 pi
        if(x != 0 || y != 0)
        {
            if(i == 1 || i == 4)
            {
                if(Math.abs(x) > 0) {
                    if(Math.abs(x) > Math.abs(y)) {
                        double t = TrueSign(x);
                        movePower = t;
                    } else {
                        //If y is positive returns 1, if y is negative returns -1, if y is 0 returns 0
                        double t = TrueSign(y);
                        movePower = t;
                    }
                } else if(Math.abs(y) > 0) {
                    double t = TrueSign(y);
                    movePower = t;
                }
            }
            if(i == 2 || i == 3) {
                if(Math.abs(x) > 0) {
                    if(Math.abs(x) > Math.abs(y)) {
                        double t = TrueSign(x);
                        movePower = -t;
                    } else {
                        //If y is positive returns 1, if y is negative returns -1, if y is 0 returns 0
                        double t = TrueSign(y);
                        movePower = t;
                    }
                } else if(Math.abs(y) > 0) {
                    double t = TrueSign(y);
                    movePower = t;
                }
            }
        }

        //z is just the magnitude of stick move
        movePower *= z;

        double rZ = abs(x2);

        //something up with rotation, what likely will need to do is control each wheel seperately
        double rotationPower = 0;
        if(x2 != 0) {
            if(i == 1 || i == 3) {
                if(x2 < 0) {
                    rotationPower = -1;
                } else if(x2 > 0) {
                    rotationPower = 1;
                } else {
                    rotationPower = 0;
                }
            } else if (i == 2 || i == 4) {
                if(x2 < 0) {
                    rotationPower = 1;
                } else if(x2 > 0) {
                    rotationPower = -1;
                } else {
                    rotationPower = 0;
                }
            }
        }

        rotationPower *= rZ;

        double power;
        if(movePower != 0 && rotationPower == 0) {
            power = movePower;
        } else if(movePower == 0 && rotationPower != 0) {
            power = rotationPower;
        } else {
            power = (double)((movePower + rotationPower) / 2);
        }

        //power *= 0.5f;

        return power;
    }

    //Manipulate values in here in force matrix
    public double GetWheelForceTank(double y, int i, float x2)
    {
        if(y == 0 && x2 == 0) {
            return 0;
        }

        double movePower = 0;


        //FORCE MATRIX i == 1 is left_front i == 2 is right_front i == 3 is left_back i == 4 is right_back
        //Uses unit circle math look it up if you need to
        //theta is just divided by pi so 2 is equal in angle to 2 pi
        if(y != 0)
        {
            movePower = y;
        }

        double rZ = abs(x2);

        //something up with rotation, what likely will need to do is control each wheel seperately
        double rotationPower = 0;
        if(x2 != 0) {
            if(i == 1 || i == 3) {
                if(x2 < 0) {
                    rotationPower = -1;
                } else if(x2 > 0) {
                    rotationPower = 1;
                } else {
                    rotationPower = 0;
                }
            } else if (i == 2 || i == 4) {
                if(x2 < 0) {
                    rotationPower = 1;
                } else if(x2 > 0) {
                    rotationPower = -1;
                } else {
                    rotationPower = 0;
                }
            }
        }

        rotationPower *= rZ;

        double power;
        if(movePower != 0 && rotationPower == 0) {
            power = movePower;
        } else if(movePower == 0 && rotationPower != 0) {
            power = rotationPower;
        } else {
            power = (double)((movePower + rotationPower) / 2);
        }

        //power *= 0.5f;

        return power * controlMultiplier;
    }


    /*
    Angle THETA is found by the arctan of the controller y / x. We then further divide this by PI
    to reduce further computations (i.e. PI / 2 -> 1/2)
     */

    double angleAdder = 0;
    public double Theta(double x, double y, int q)
    {
        angleAdder = 0;
        double iAngle = (Math.PI * q / 2 + angleAdder);

        if(x == 0) {
            return iAngle / Math.PI;
        } else {
            double div = abs(y) / abs(x);
            double angle = Math.atan(abs(div)) + iAngle;

            if(angle >= 2 * Math.PI) {
                angle %= 2 * Math.PI;
            }

            return angle / Math.PI;
        }
    }

    public int GetQuad(double x, double y) {

        if(x == 0 && y == 0)
            return 0;
        else if(x > 0 && y > 0)
            return 0;
        else if(x < 0 && y > 0)
            return 1;
        else if(x < 0 && y < 0)
            return 2;
        else if(x > 0 && y < 0)
            return 3;
        else if(x > 0 && y == 0)
            return 0;
        else if(x == 0 && y > 0)
            return 0;
        else if(x < 0 && y == 0)
            return 1;
        else if(x == 0 && y < 0)
            return 2;
        else
            return 0;
    }

    public final double[] rangeQ1 = new double[] {0, Math.PI};
    public final double[] rangeQ2 = new double[] {-5 * Math.PI / 6, 0};
    public final double[] rangeQ3 = new double[] {0, Math.PI}; //Change?? Test with servo controller
    //public final double q3Offset = -Math.PI;
    double pastX = 0, pastY = 0, pastPhi = 0;

    public double tQ1 = 0, tQ2 = 0, tQ3 = 0;
    public double[] IKArm(double x, double y, double phi) {
        double finalQ1 = currentQ1, finalQ2 = currentQ2, finalQ3 = currentQ3;
        double[] endArray = new double[] {finalQ1, finalQ2, finalQ3};
        if(x == pastX && y == pastY && phi == pastPhi) {
            return endArray;
        }
        pastX = x;
        pastY = y;

        //Angle of the secondary elbow joint needed to be found first
        double q2Top = Math.pow(x,2) + Math.pow(y,2) - Math.pow(lengthOfFirstArmJoint,2) - Math.pow(lengthOfSecondArmJoint,2);
        double q2Bottom = 2 * lengthOfFirstArmJoint * lengthOfSecondArmJoint;
        double q2 = -Math.acos(q2Top / q2Bottom);
        //no need to be concerned about +q2 with clamps because both will be clamped, need to normalize it back into 0-1 space, but
        //the servo is reversed
        finalQ2 = Math.abs(Clamp(q2, rangeQ2[0], rangeQ2[1]));

        //Angle of the base joint needs to have q2 found
        double q1 = Math.atan2(y, x) - Math.atan2(lengthOfFirstArmJoint * Math.sin(q2), lengthOfFirstArmJoint + lengthOfSecondArmJoint * Math.cos(q2));
        finalQ1 = Clamp(q1, rangeQ1[0], rangeQ1[1]);

        //Angle of the hand joint, needs q1 and q2 to be known
        double q3 = phi - q2 - q1;
        finalQ3 = Clamp(q3, rangeQ3[0], rangeQ3[1]);

        tQ1 = q1; tQ2 = q2; tQ3 = q3;

        currentQ1 = finalQ1; currentQ2 = finalQ2; currentQ3 = finalQ3;
        return new double [] {finalQ1, finalQ2, finalQ3};
    }

    public double[] IKTargetClamp(double x_, double y_)
    {
        double x;
        double y = y_;

        if(y <= 0) {
            double xMin = lengthOfFirstArmJoint - lengthOfSecondArmJoint * cos(Math.PI / 6);
            double xMax = lengthOfFirstArmJoint + lengthOfSecondArmJoint;
            x = Clamp(x_, xMin, xMax);

            double yMin = Math.sqrt(Math.pow(lengthOfSecondArmJoint, 2) - Math.pow(x, 2));
            double yMax = 0;
            y = Clamp(y_, yMin, yMax);
        } else {
            double xMin = -lengthOfFirstArmJoint - lengthOfSecondArmJoint;
            double xMax = lengthOfFirstArmJoint + lengthOfSecondArmJoint;
            x = Clamp(x_, xMin, xMax);

            double yMin = 0;
            double yMax = Math.sqrt(Math.pow(lengthOfSecondArmJoint + lengthOfFirstArmJoint, 2) - Math.pow(x, 2));
            y = Clamp(y_, yMin, yMax);
        }

        return new double[] {x, y};
    }

    public void SetMultiplier(float x) {
        controlMultiplier = x;
    }

    public int TrueSign(double n) {
        if(n > 0) {
            return 1;
        } else if (n == 0) {
            return 0;
        } else {
            return - 1;
        }
    }

    public double Clamp(double value, double min, double max) {
        if(value > max) {
            return max;
        } else if(value < min) {
            return min;
        } else {
            return value;
        }
    }
}


