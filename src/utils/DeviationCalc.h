/*
 * DeviationCalc.h
 *
 *  Created on: 17 марта 2015 г.
 *      Author: Max
 */

#ifndef DEVIATIONCALC_H_
#define DEVIATIONCALC_H_

#include <math.h>

namespace ev3way {

class DeviationCalc {
private:
    float average = 0;
    float variance = 0;
    int count = 0;

public:
    void add(float value) {
        ++count;
        float prior_average = average;
        average = average + (value - average) / count;
        variance = variance + (value - prior_average)*(value - average);
    }

    float getAverage() const {
        return average;
    }

    float getDeviation() const {
        return (float) sqrt(variance / count);
    }
};

} /* namespace ev3way */

#endif /* DEVIATIONCALC_H_ */
