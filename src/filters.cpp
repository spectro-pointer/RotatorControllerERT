#include "filters.h"

SecondOrderLowPassFilter::SecondOrderLowPassFilter(double cutoffFreq, double sampleRate) {
    this->cutoffFreq = cutoffFreq;
    this->sampleRate = sampleRate;

    yPrev = yPrev2 = xPrev = xPrev2 = 0.0;
}


double SecondOrderLowPassFilter::process(double input) {
    double omega = 2.0 * M_PI * cutoffFreq / sampleRate;
    double alpha = sin(omega) / (2.0 * 0.7071); // 0.7071 is the damping ratio for a Butterworth filter

    double b0 = (1.0 - cos(omega)) / 2.0;
    double b1 = 1.0 - cos(omega);
    double b2 = (1.0 - cos(omega)) / 2.0;
    double a0 = 1.0 + alpha;
    double a1 = -2.0 * cos(omega);
    double a2 = 1.0 - alpha;

    double output = (b0 / a0) * input + (b1 / a0) * xPrev + (b2 / a0) * xPrev2
                    - (a1 / a0) * yPrev - (a2 / a0) * yPrev2;

    // Update state variables
    xPrev2 = xPrev;
    xPrev = input;
    yPrev2 = yPrev;
    yPrev = output;

    return output;
}

void SecondOrderLowPassFilter::reset() {
    yPrev = yPrev2 = xPrev = xPrev2 = 0.0;
}

void SecondOrderLowPassFilter::reset(double initialValue) {
    yPrev = yPrev2 = xPrev = xPrev2 = initialValue;
}

void SecondOrderLowPassFilter::setCutoffFreq(double cutoffFreq) {
    this->cutoffFreq = cutoffFreq;
}



SecondOrderEstimator::SecondOrderEstimator() {
    xPrev = xPrevTime = xPrev2 = xPrev2Time = sampleCount = 0.0;
}

void SecondOrderEstimator::update(double input, unsigned long timeMillis) {
    xPrev2 = xPrev;
    xPrev2Time = xPrevTime;
    xPrev = x;
    xPrevTime = xTime;
    x = input;
    xTime = timeMillis;

    sampleCount++;
}

double SecondOrderEstimator::computeAngle(unsigned long timeMillis) {
    if (sampleCount < 3) {
        return x;
    }
    else if (sampleCount > 3 and (timeMillis - xTime) < maxTimeWindow) {
        //Serial.println("In the window");
        double xDot = getAngleFilter(xPrev,x) / ((xTime - xPrevTime) / 1000.0);
        double xDotPrev = getAngleFilter(xPrev2,xPrev) / ((xPrevTime - xPrev2Time) / 1000.0);
        double xDotDot = (xDot - xDotPrev) / ((xTime - xPrevTime) / 1000.0);
        return x+xDot*(timeMillis-xTime)/1000.0+0.5*xDotDot*pow((timeMillis-xTime)/1000.0,2);
    }
    else {
        //Serial.println("Out of the window");
        double xDot = getAngleFilter(xPrev,x) / ((xTime - xPrevTime) / 1000.0);
        double xDotPrev = getAngleFilter(xPrev2,xPrev) / ((xPrevTime - xPrev2Time) / 1000.0);
        double xDotDot = (xDot - xDotPrev) / ((xTime - xPrevTime) / 1000.0);
        return x+xDot*(maxTimeWindow)/1000.0+0.5*xDotDot*pow((maxTimeWindow)/1000.0,2);
    }
}

double SecondOrderEstimator::compute(unsigned long timeMillis) {
    if (sampleCount < 3) {
        return x;
    }
    else if (sampleCount > 3 and (timeMillis - xTime) < maxTimeWindow) {
        //Serial.println("In the window");
        double xDot = x-xPrev / ((xTime - xPrevTime) / 1000.0);
        double xDotPrev = xPrev-xPrev2 / ((xPrevTime - xPrev2Time) / 1000.0);
        double xDotDot = (xDot - xDotPrev) / ((xTime - xPrevTime) / 1000.0);
        return x+xDot*(timeMillis-xTime)/1000.0+0.5*xDotDot*pow((timeMillis-xTime)/1000.0,2);
    }
    else {
        //Serial.println("Out of the window");
        double xDot = x-xPrev / ((xTime - xPrevTime) / 1000.0);
        double xDotPrev = xPrev-xPrev2 / ((xPrevTime - xPrev2Time) / 1000.0);
        double xDotDot = (xDot - xDotPrev) / ((xTime - xPrevTime) / 1000.0);
        return x+xDot*(maxTimeWindow)/1000.0+0.5*xDotDot*pow((maxTimeWindow)/1000.0,2);
    }
}

void SecondOrderEstimator::reset() {
    x = xPrev = xPrev2 = 0.0;
    xTime = xPrevTime = xPrev2Time = 0;
    sampleCount = 0;
}

void SecondOrderEstimator::reset(double initialValue) {
    x = xPrev = xPrev2 = initialValue;
    xTime = xPrevTime = xPrev2Time = 0;
    sampleCount = 0;
}

void SecondOrderEstimator::setMaxTimeWindow(unsigned long maxTimeWindowIn) {
    maxTimeWindow = maxTimeWindowIn;
}

unsigned long SecondOrderEstimator::getMaxTimeWindow() {
    return maxTimeWindow;
}

double getAngleFilter(double angle1, double angle2) {
    double angle = angle2 - angle1;
    if (angle > 180) {
        angle -= 360;
    }
    if (angle < -180) {
        angle += 360;
    }
    return angle;
}

