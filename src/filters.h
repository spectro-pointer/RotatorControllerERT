#include <Arduino.h>

class SecondOrderLowPassFilter {
    private:
        double yPrev;    // Previous output
        double yPrev2;   // Output before previous output
        double xPrev;    // Previous input
        double xPrev2;   // Input before previous input

        double cutoffFreq; // Cutoff frequency of the filter
        double sampleRate; // Sampling rate

    public:
        SecondOrderLowPassFilter(double cutoffFreq, double sampleRate);
        double process(double input);
        void reset();
        void reset(double initialValue);
        void setCutoffFreq(double cutoffFreq);
};

class SecondOrderEstimator {
    private:
        double x;
        double xTime;
        double xPrev;       // Previous input
        double xPrevTime;   // Previous input time    
        double xPrev2;      // Input before previous input
        double xPrev2Time;  // Input before previous input time

        int sampleCount;
        unsigned long maxTimeWindow;

        double maxSpeed;
        double maxAccel;

    public:
        SecondOrderEstimator();
        void update(double input, unsigned long timeMillis);
        double computeAngle(unsigned long timeMillis);
        double compute(unsigned long timeMillis);
        void reset();
        void reset(double initialValue);
        void setMaxTimeWindow(unsigned long maxTimeWindowIn);
        unsigned long getMaxTimeWindow();
        void setMaxSpeed(double maxSpeedIn);
        void setMaxAccel(double maxAccelIn);
};

double getAngleFilter(double angle1, double angle2);
