#include <math.h>

namespace markerStats
{
    const double borderRatio = 0.045;
    const double rhombusRatio = 0.85;
    const double rhombusLength = (sqrt(2) * rhombusRatio) / 2;
    const double outerCircleRadius = (0.95 * rhombusLength) / 2;
    const double innerCircleRadius = 0.875 * outerCircleRadius;
}