package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
/*import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;*/

import static java.lang.Math.abs;

public class Algorithms001 {

    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = 6 * mmPerInch;

    public static final float halfField = 72 * mmPerInch;
    public static final float quadField = 36 * mmPerInch;
    
    //Default control multiplier is 0.2, increase to make the robot faster, decrease to make the robot slower.
    float controlMultiplier = 0.2f;

    public static final float wheelCircumferenceMm = 301.59f;
    public static final float rotationPerOneRevolution = 2.314f;

    /*public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        //Converts float[] to OpenGLMatrix.
        OpenGLMatrix trans = OpenGLMatrix.translation(x, y, z);

        if(trans != null) {
            OpenGLMatrix o = Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, u, v, w);

            if(o != null) {
                OpenGLMatrix multiplied = null;

                if(trans!= null)
                    multiplied = trans.multiplied(o);

                if(multiplied != null)
                    return multiplied;
                else
                    return null;
            } else
                return null;
        }
        return null;
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
           if(y > 0) {
               movePower = 1;
           } else if(y < 0) {
               movePower = -1;
           }
        }

        //z is just the magnitude of stick move
        movePower *= Math.abs(y);

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

    public void SetMultiplier(float x) {
        controlMultiplier = x;
    }

    float angleAdder = 0f;
    public void SetDirection(float _t)
    {
        angleAdder = _t;
    }

    public int Sign(double n) {
        if(n >= 0) {
            return 1;
        } else {
            return -1;
        }
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

    float margin = 0.5f;
    /*public boolean isInShooterPosBlueTower(OpenGLMatrix transform) {
        //A range (parabolic) of positions that the root can inhabit and shoot in
        float[] data = null;

        if(transform != null)
            data = transform.getData();

        if(data == null) return false;

        float fiveE = quadField / 2;
        float sevenE = quadField + quadField / 2;

        float x = data[0];
        float y = data[0];

        float oneFoot = 12 * mmPerInch;

        if(!(x > -quadField - (oneFoot / 4) && x < -quadField + (oneFoot / 4))) {
            return false;
        }

        float correctYPos = ShootParabola(x - quadField);

        if(!(y > -quadField - (oneFoot / 4) && y < -quadField + (oneFoot / 4))) {
            return false;
        }

        return true;
    }*/

    public float DistanceToTargetBlue(float currentPos) {
        return -quadField - currentPos;
    }

    float AngleToBlueShootParabola(float currentAngle) {
        return  0;
    }

    float ShootParabola(float x) {
        return (x*x)/100 - 1.55525f;
    }

    /*public double GetDistance(OpenGLMatrix one, OpenGLMatrix two) {
        //Simple distance function

        VectorF t1 = one.getTranslation();
        VectorF t2 = two.getTranslation();
        return Math.sqrt(Math.pow((t1.get(0) - t2.get(0)), 2) + Math.pow((t1.get(1) - t2.get(1)), 2));
    }

    //Display a OpenGLMatrix
    public String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }*/
}

