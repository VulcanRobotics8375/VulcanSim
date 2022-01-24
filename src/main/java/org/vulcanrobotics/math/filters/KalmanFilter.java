package org.vulcanrobotics.math.filters;

public class KalmanFilter {
    private double prioriEstimate;
    private double posterioriEstimate;
    //Kalman Gain - 0 <= K <= 1 - tells us how much we want to change our estimate by a given measurement.
    private double gain;
    //Uncertianty in Estimate
    private double variance;
    private double initialEstimate;
    private double initialVariance;
    private double velocity;
    //Outside noise - such as due to friction or wind speed
    private double noise;
    //Uncertainty in the measurement
    private double uncertainty;
    private int n = 0;

    public KalmanFilter(double initialEstimate, double initialVariance, double noise){
        this.initialEstimate = initialEstimate;
        this.initialVariance = initialVariance;
        this.noise = noise;
    }

    public double run(double measurement, double uncertainty){
        if(n == 0){
            variance = initialVariance + noise;
            prioriEstimate = initialEstimate;
            n++;
        }else{
            gain = variance / (variance + uncertainty);
            posterioriEstimate = prioriEstimate + gain * (measurement - prioriEstimate);
            variance = (1 - gain) * variance + noise;
        }
        return posterioriEstimate;
    }
}
