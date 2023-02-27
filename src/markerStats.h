#include <math.h>

namespace markerStats
{
    const double borderRatio = 0.045;
    const double diamondRatio = 0.85;
    const double diamondLength = (sqrt(2) * diamondRatio) / 2;
    const double outerCircleRadius = (0.95 * diamondLength) / 2;
    const double innerCircleRadius = 0.875 * outerCircleRadius;
}