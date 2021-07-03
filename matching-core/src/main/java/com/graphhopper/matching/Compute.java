package com.graphhopper.matching;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.linear.*;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class Compute {

    private RealMatrix E;
    private RealMatrix S;
    private RealMatrix STranspose;
    private RealMatrix H;
    private RealMatrix A;
    private double b = 0.0;
    private double lambda1 = 15;//权重w的惩罚项
    private double lambda2 = 1.5;//概率p的惩罚项
    private int candidateNum;
    private static int metrics;

    public static RealVector getResult(double[][] mat, int candidateNum, int metric) {
        metrics = metric;
        Compute compute = new Compute();
        compute.init(mat, candidateNum);
        return compute.start();
    }

    //int m=0;

    void init(double[][] mat, int candidateNum) {
        this.candidateNum = candidateNum;
//        Give value to E;
        double[][] EArrays = new double[2][metrics + candidateNum];
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < metrics + candidateNum; j++) {
                EArrays[i][j] = ((i == 0 && j < metrics) || (i == 1 && j >= metrics)) ? 1 : 0;
            }
        }
        E = MatrixUtils.createRealMatrix(EArrays);

//        Random Create S
        double[][] SArrays = new double[mat.length][mat[0].length];


        for(int i = 0; i < metrics; i++) {
            for(int j = 0; j < candidateNum; j++) {
                SArrays[i][j] =  mat[i][j];
            }
        }
        S = MatrixUtils.createRealMatrix(SArrays);


        STranspose = S.transpose();
        double[][] STArrays = STranspose.getData();

        double[][] HArrays = new double[metrics + candidateNum][metrics+candidateNum];
        for(int i = 0; i < metrics + candidateNum; i++) {
            for(int j = 0; j < metrics + candidateNum; j++) {
                if(i < metrics && j < metrics) {
                    HArrays[i][j] = (i == j) ? lambda1 : 0;//权重惩罚项
                }
                else if(j >=  metrics && i < metrics) {
                    HArrays[i][j] = -0.5 * SArrays[i][j-metrics];
                }
                else if(j < metrics && i >= metrics) {
                    HArrays[i][j] = -0.5 * STArrays[i-metrics][j];
                }
                else {
                    HArrays[i][j] = (i == j) ? lambda2 : 0;//概率惩罚项
                }
            }
        }

        H = MatrixUtils.createRealMatrix(HArrays);

        double[] AArrays = new double[metrics + candidateNum];
        Arrays.fill(AArrays, 1.0);
        A = MatrixUtils.createRealDiagonalMatrix(AArrays);
    }


    public RealVector start() {
        double[] xArray = new double[metrics + candidateNum];
        xArray[metrics-1] = 1;
        xArray[metrics + candidateNum - 1] = 1;
        RealVector x = MatrixUtils.createRealVector(xArray);
        PointValuePair solution = null;

        for(int iteration = 0; iteration < 10; iteration++) {
            List<double[]> effectiveConstrain = new ArrayList<>();
            List<double[]> nonEffectiveConstrint = new ArrayList<>();
            for(int i = 0; i < metrics + candidateNum; i++) {
                if(Math.abs(x.dotProduct(A.getRowVector(i))) < 1e-5) {
                    effectiveConstrain.add(A.getRow(i));
                } else {
                    nonEffectiveConstrint.add(A.getRow(i));
                }
            }

            double[][] a1Arrays = new double[effectiveConstrain.size()][metrics+candidateNum];
            double[][] a2Arrays = new double[nonEffectiveConstrint.size()][metrics + candidateNum];

            for(int i = 0; i < effectiveConstrain.size(); i++) {
                for(int j = 0; j < metrics + candidateNum; j++) {
                    a1Arrays[i][j] = effectiveConstrain.get(i)[j];
                }
            }

            for(int i = 0; i < nonEffectiveConstrint.size(); i++) {
                for(int j = 0; j < metrics + candidateNum; j++) {
                    a2Arrays[i][j] = nonEffectiveConstrint.get(i)[j];
                }
            }

            List<LinearConstraint> constraints = new ArrayList<>();

            if(effectiveConstrain.size() > 0) {
                RealMatrix A1 = MatrixUtils.createRealMatrix(a1Arrays);
                for(int i = 0; i < A1.getRowDimension(); i++) {
                    constraints.add(new LinearConstraint(A1.getRow(i), Relationship.GEQ, 0.0));
                }

            }


            RealMatrix A2 = MatrixUtils.createRealMatrix(a2Arrays);


            for(int i = 0; i < 2; i++) {
                constraints.add(new LinearConstraint(E.getRowVector(i), Relationship.EQ, 0));
            }

            for(int i = 0; i < metrics + candidateNum; i++) {
                double[] arr = new double[metrics + candidateNum];
                Arrays.fill(arr, 0.0);
                arr[i] = 1;
                constraints.add(new LinearConstraint(arr, Relationship.GEQ, -1.0));
                constraints.add(new LinearConstraint(arr, Relationship.LEQ, 1.0));
            }


            RealVector xtH = H.scalarMultiply(2).operate(x);

            LinearObjectiveFunction f = new LinearObjectiveFunction(xtH, 0);
            try {
                solution = new SimplexSolver().optimize(f, new LinearConstraintSet(constraints), GoalType.MINIMIZE);
            }
            catch (Exception e) {
                e.printStackTrace();
            }

            if (solution != null) {
                //get solution
                double max = solution.getValue();

                if(Math.abs(max) > 1e-5) {
                    double miuMax = 0.0;
                    boolean isAllBiggerThan = true;
                    for(int i = 0; i < A2.getRowDimension(); i++) {
                        if(A2.getRowVector(i).dotProduct(MatrixUtils.createRealVector(solution.getPoint())) < 0.0) {
                            isAllBiggerThan = false;
                            break;
                        }
                    }
                    if(isAllBiggerThan) {
                        miuMax = Double.POSITIVE_INFINITY;
                    } else {
                        double[] b2 = new double[A2.getRowDimension()];
                        double[] toSub = A2.operate(x.toArray());
                        RealVector upper = MatrixUtils.createRealVector(b2).subtract(MatrixUtils.createRealVector(toSub));
                        RealVector A2d = MatrixUtils.createRealVector(A2.operate(solution.getPoint()));
                        RealVector res = upper.ebeDivide(A2d);


                        double minValue = Double.MAX_VALUE;
                        int minInd = -1;
                        for (int i = 0; i < A2d.getDimension(); i++) {
                            if (A2d.getEntry(i) < 0.0) {
                                if (res.getEntry(i) < minValue) {
                                    minValue = res.getEntry(i);
                                    minInd = i;
                                }
                            }
                        }

                        if (minInd != -1) {
                            miuMax = minValue;
                        }
                    }

//                Calculate miu0
                    double miuUpper = MatrixUtils.createRealVector(H.preMultiply(solution.getPoint())).dotProduct(x);
                    double miuLower = MatrixUtils.createRealVector(H.preMultiply(solution.getPoint())).dotProduct(MatrixUtils.createRealVector(solution.getPoint()));
                    double miu0 = - miuUpper / miuLower;

                    double[] miuList;

                    if(miu0 > miuMax || miu0 < 0) {
                        miuList = new double[]{0.0, miuMax};
                    } else {
                        miuList = new double[]{0.0, miuMax, miu0};
                    }

                    List<Double> value = new ArrayList<>();
                    for(double mu : miuList) {
                        RealVector vector = x.add(MatrixUtils.createRealVector(solution.getPoint()).mapMultiply(mu));
                        value.add(H.preMultiply(vector).dotProduct(vector));
                    }

                    int valueMinInd = -1;
                    double valueMinValue = Double.MAX_VALUE;
                    for(int i = 0; i < value.size(); i++) {
                        if(valueMinValue > value.get(i)) {
                            valueMinInd = i;
                            valueMinValue = value.get(i);
                        }
                    }

                    double miu = miuList[valueMinInd];
                    x = x.add(MatrixUtils.createRealVector(solution.getPoint()).mapMultiply(miu));
                } else {
                    break;
                }
            }
        }
        return x;

    }

    public static void main(String[] args) {
        Compute compute = new Compute();
//        compute.init(2);
        compute.start();
    }
}
