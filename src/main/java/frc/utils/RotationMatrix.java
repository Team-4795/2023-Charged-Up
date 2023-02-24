package frc.utils;

public class RotationMatrix {
    //rotation[row][column]
    double[][] rotation;
    //solutionSet[row][y or z coefficient] -> for index #2: 0 - y, 1 - z
    double[][] solutionSet;
    double[] intersectBasis;

    double[][] opMatrix;

    public RotationMatrix(){
        rotation = new double[3][3];
        solutionSet = new double[3][2];
        opMatrix = new double[3][5];
        resetSolution();
    }

    public RotationMatrix(double x, double y, double z){
        rotation = new double[3][3];
        solutionSet = new double[3][2];
        opMatrix = new double[3][5];
        setRotationMatrix(x, y, z);
        resetSolution();
    }
    
    public void setRotationMatrix(double x, double y, double z){
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

    public void resetSolution(){
        solutionSet[0][0] = 0;
        solutionSet[0][1] = 0;
        solutionSet[1][0] = 1;
        solutionSet[1][1] = 0;
        solutionSet[2][0] = 0;
        solutionSet[2][1] = 1;
    }

    public void getSolution(){
        setOperationMatrix();
        int iteration = 0;
        int[] pivotCoords = new int[2];
        while(!isRREF()){
            pivotCoords = pivotInterchange(iteration);
            scaleRow(pivotCoords[0], (1 / opMatrix[pivotCoords[0]][pivotCoords[1]]));
            elimColumn(pivotCoords[1]);
            sortZeroRows();
            iteration++;
        }

    }

    private void setOperationMatrix(){
        for(int i = 0; i < rotation.length; i++){
            for(int j = 0; j < rotation[i].length; j++){
                opMatrix[i][j] = rotation[i][j];
            }
        }
        for(int i = 0; i < opMatrix.length; i++){
            for(int j = rotation[i].length; j < opMatrix[i].length; j++){
                opMatrix[i][j] = solutionSet[i][j];
            }
        }
    }

    private int[] pivotInterchange(int iteration){
        boolean found = false;
        int pivotRow = -1;
        int pivotColumn = -1;
        for(int j = iteration; j < opMatrix[0].length; j++){
            for(int i = iteration; i < opMatrix.length; i++){
                if(opMatrix[i][j] != 0 && !found){
                    pivotRow = i;
                    pivotColumn = j;
                    found = true;
                }
                break;
            }
            if(found){
                break;
            }
        }
        if(pivotRow != iteration){
            rowChange(pivotRow, iteration);
        }
        return new int[]{pivotRow, pivotColumn};
    }

    private void elimColumn(int column){

    }

    private void sortZeroRows(){

    }

    private boolean isRREF(){
        //only first three columns
        return true;
    }

    private void rowChange(int row1, int row2){
        double[] hold = new double[opMatrix[row1].length];
        System.arraycopy(row1, 0, hold, 0, opMatrix[row1].length);
        System.arraycopy(opMatrix[row2], 0, opMatrix[row1], 0, hold.length);
        System.arraycopy(hold, 0, opMatrix[row2], 0, hold.length);
    }

    private void rowAddition(int replaceRow, double scalar, int addRow){
        double[] hold = new double[opMatrix[addRow].length];
        System.arraycopy(opMatrix[addRow], 0, hold, 0, opMatrix[addRow].length);
        for(int i = 0; i < hold.length; i++){
            hold[i] *= scalar;
            hold[i] += opMatrix[replaceRow][i];
        }
        System.arraycopy(hold, 0, opMatrix[replaceRow], 0, hold.length);
    }

    private void scaleRow(int row, double scalar){
        double[] hold = new double[opMatrix[row].length];
        System.arraycopy(opMatrix[row], 0, hold, 0, opMatrix[row].length);
        for(int i = 0; i < hold.length; i++){
            hold[i] *= scalar;
        }
        System.arraycopy(hold, 0, opMatrix[row], 0, hold.length);
    }

}   
