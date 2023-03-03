package frc.utils;

import frc.robot.Constants.AutoConstants;

public class RotationMatrix {
    //rotation[row][column]
    double[][] rotation;
    double[][] incremented;

    //u^(1/3) is approx. 0.000006055
    final double increment = 0.000007;

    //true if vector [0, 1, 0] is forward, false if [1, 0, 0] is forward
    final boolean yForward = true;

    double[] solutionSet;

    public RotationMatrix(){
        rotation = new double[3][3];
        solutionSet = new double[3];
        incremented = new double[3][3];
    }

    public RotationMatrix(double x, double y, double z){
        rotation = new double[3][3];
        solutionSet = new double[3];
        incremented = new double[3][3];
        setRotationMatrix(x, y, z);
    }

    //finds elevation angle using the rotation matrix
    public double findElevationAngle(double x1, double y1, double z1){
        setRotationMatrix(x1, y1, z1);
        double x = 0;
        double y = 0;
        if(yForward){
            if(Math.abs(rotation[0][1]) < AutoConstants.toZeroBound){
                //assumption that if y == NaN, then no elevation angle exists
                return 0.0;
            } else {
                x = 1;
                y = -rotation[0][0] / rotation[0][1];
            }
            multiply(new double[]{x, y, 0}, rotation);
            return Math.toDegrees(Math.atan2(solutionSet[2], solutionSet[1]));
        } else {
            if(Math.abs(rotation[1][1]) < AutoConstants.toZeroBound){
                return 0.0;
            } else {
                x = 1;
                y = -rotation[1][0] / rotation[1][1];
            }
            multiply(new double[]{x, y, 0}, rotation);
            return Math.toDegrees(Math.atan2(solutionSet[2], solutionSet[0]));
        }
    }

    //utilizes the approx. for d/dt(f(x1, y1, z1)) = f_x(x1, y1, z1) * dx/dt + f_y(x1, y1, z1) * dy/dt + f_z(x1, y1, z1) * dz/dt
    public double findElevationVelocity(double x, double y, double z, double dX, double dY, double dZ){
        double[] differentials = new double[3];
        for(int i = 0; i < differentials.length; i++){
            //f(x+h) - f(x-h)/2h    
            setIncrementMatrix(x, y, z, true, i);
            differentials[i] = findElevationIncrement();
            setIncrementMatrix(x, y, z, false, i);
            differentials[i] = (differentials[i] - findElevationIncrement())/(2 * increment);
        }
        return (differentials[0] * dX + differentials[1] * dY + differentials[2] * dZ);
    }
    
    //sets the matrix used for elevation angle calculations
    private void setRotationMatrix(double x, double y, double z){
        x = Math.toRadians(x);
        y = Math.toRadians(y);
        z = Math.toRadians(z);
        rotation[0][0] = Math.cos(y) * Math.cos(z);
        rotation[0][1] = Math.sin(x) * Math.sin(y) * Math.cos(z) - Math.cos(x) * Math.sin(z);
        rotation[0][2] = Math.cos(x) * Math.sin(y) * Math.cos(z) - Math.sin(x) * Math.sin(z);
        rotation[1][0] = Math.sin(z) * Math.cos(y);
        rotation[1][1] = Math.sin(x) * Math.sin(y) * Math.sin(z) + Math.cos(x) * Math.cos(z);
        rotation[1][2] = Math.cos(x) * Math.sin(y) * Math.sin(z) - Math.sin(x) * Math.cos(z);
        rotation[2][0] = -Math.sin(y);
        rotation[2][1] = Math.sin(x) * Math.cos(y);
        rotation[2][2] = Math.cos(x) * Math.cos(y);
    }

    //sets the matrix used for derivative approx.
    private void setIncrementMatrix(double x, double y, double z, boolean positive, int index){
        x = Math.toRadians(x);
        y = Math.toRadians(y);
        z = Math.toRadians(z);
        switch(index){
            case 0:
                if(positive){
                    x += increment; 
                    break;
                } else {
                    x -= increment;
                    break;
                }
            case 1:
                if(positive){
                    y += increment; 
                    break;
                } else {
                    y -= increment;
                    break;
                }
            case 2:
                if(positive){
                    z += increment; 
                    break;
                } else {
                    z -= increment;
                    break;
                }
        }
        incremented[0][0] = Math.cos(y) * Math.cos(z);
        incremented[0][1] = Math.sin(x) * Math.sin(y) * Math.cos(z) - Math.cos(x) * Math.sin(z);
        incremented[0][2] = Math.cos(x) * Math.sin(y) * Math.cos(z) - Math.sin(x) * Math.sin(z);
        incremented[1][0] = Math.sin(z) * Math.cos(y);
        incremented[1][1] = Math.sin(x) * Math.sin(y) * Math.sin(z) + Math.cos(x) * Math.cos(z);
        incremented[1][2] = Math.cos(x) * Math.sin(y) * Math.sin(z) - Math.sin(x) * Math.cos(z);
        incremented[2][0] = -Math.sin(y);
        incremented[2][1] = Math.sin(x) * Math.cos(y);
        incremented[2][2] = Math.cos(x) * Math.cos(y);
    }

    //finds f(x +/- h)
    private double findElevationIncrement(){
        double x = 0;
        double y = 0;
        if(yForward){
            if(Math.abs(incremented[0][1]) < AutoConstants.toZeroBound){
                //this could honestly cause problems with the derivative approx. bc returning 0 is a mere convenient guess, not mathematically proven
                return 0.0;
            } else {
                x = 1;
                y = -incremented[0][0] / incremented[0][1];
            }
            multiply(new double[]{x, y, 0}, incremented);
            return Math.toDegrees(Math.atan2(solutionSet[2], solutionSet[1]));
        } else {
            if(Math.abs(incremented[1][1]) < AutoConstants.toZeroBound){
                //same as above^^^^
                return 0.0;
            } else {
                x = 1;
                y = -incremented[1][0] / incremented[1][1];
            }
            multiply(new double[]{x, y, 0}, incremented);
            return Math.toDegrees(Math.atan2(solutionSet[2], solutionSet[0]));
        }
    }

    //multiplying a vector by a transformation matrix
    private void multiply(double[] vector, double[][] base){
        double sum;
        for(int i = 0; i < vector.length; i++) {
            sum = 0;
            for (int j = 0; j < base[i].length; j++) {
                sum += vector[j] * base[i][j];
            }
            solutionSet[i] = sum;
        }
    }

}   
