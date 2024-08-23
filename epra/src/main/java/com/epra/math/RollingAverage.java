package com.epra.math;

import java.util.ArrayList;

/**A rolling average of values stored in a buffer.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.
 * <p></p>
 * This class is intended for use sensors that give sometimes unreliable outputs. This average should flatten out any outliers.
 * The average can be biased towards more recent values so that actual movement is preserved.*/
public class RollingAverage {

    /**Enum containing all types of bias the rolling average can use and the corresponding methods for those bias types.*/
    public enum Bias {
        SIGMOID(RollingAverage::sigmoidBias),
        REVERSE_SIGMOID(RollingAverage::reverseSigmoidBias),
        LINEAR(RollingAverage::linearBias),
        REVERSE_LINEAR(RollingAverage::reverseLinearBias),
        FLAT(RollingAverage::flatBias);

        BiasPointer use;

        Bias(BiasPointer pointer) { this.use = pointer; }
    }

    private ArrayList<Double> buffer;
    private int bufferSize;
    private Bias biasType;

    /**A rolling average of values stored in a buffer.
     *<p></p>
     *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.
     * <p></p>
     * This class is intended for use sensors that give sometimes unreliable outputs. This average should flatten out any outliers.
     * The average can be biased towards more recent values so that actual movement is preserved.
     * @param bufferSize Size of the buffer.
     * @param biasType Type of bias to use in averages.*/
    public RollingAverage(int bufferSize, Bias biasType) {
        buffer = new ArrayList<Double>();
        this.bufferSize = bufferSize;
        this.biasType = biasType;
    }

    /**@param bufferSize New size of the buffer.*/
    public void setBufferSize(int bufferSize) { this.bufferSize = bufferSize; }
    /**@param biasType New type of bias to use in averages.*/
    public void setBiasType(Bias biasType) { this.biasType = biasType; }

    /**Creates a sigmoid bias multiplier between 0.0 and 2.0 so that more recent values are given more weight than older values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double sigmoidBias(int total, int recency) { return 2 / (1 + Math.pow(Math.E, ((double) (total - 1) / 2.0) - (double) (total - recency))); }
    /**Creates a sigmoid bias multiplier between 0.0 and 2.0 so that older values are given more weight than more recent values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double reverseSigmoidBias(int total, int recency) { return 2 / (1 + Math.pow(Math.E, ((double) (total - 1) / 2.0) - (double) (recency - total))); }
    /**Creates a linear bias multiplier between 0.0 and 2.0 so that more recent values are given more weight than older values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double linearBias(int total, int recency) { return 2.0 * ((double) recency / (double) total); }
    /**Creates a linear bias multiplier between 0.0 and 2.0 so that older values are given more weight than more recent values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double reverseLinearBias(int total, int recency) { return 2.0 * ((double) (total - recency) / (double) total); }
    /**Creates a flat bias of 1.0, params are not used.
     * @return 1.0*/
    public static double flatBias(int total, int recency) { return 1.0; }

    /**@param set A set of values.
     * @return The average value of the set.*/
    public double average(double[] set) {
        double v = 0.0;
        for (double d : set) { v += d; }
        return v / (double) set.length;
    }

    /**@return The weighted average value of the buffer.*/
    public double getAverage() {
        double[] set = new double[buffer.size()];
        for (int i = 0; i < set.length; i++) { set[i] = buffer.get(i) * biasType.use.bias(set.length, i); }
        return average(set);
    }
    /**@param value A value to add to the buffer.
     * @return The weighted average value of the buffer.*/
    public double addValue(double value) {
        buffer.add(value);
        if (buffer.size() > bufferSize) { buffer.remove(0); }
        return getAverage();
    }
}