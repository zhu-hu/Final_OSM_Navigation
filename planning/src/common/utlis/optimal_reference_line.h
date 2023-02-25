#ifndef OPTIMAL_REFERENCE_LINE_H
#define OPTIMAL_REFERENCE_LINE_H

#include "common/struct/reference_line.h"
#include <vector>
#include "cyber_msgs/LocalizationEstimate.h"

namespace planning{
    std::vector<ReferenceLine> OptimalReferenceLine(std::vector<ReferenceLine>& reference_lines, 
                                                    const std::vector<PredictionObstacle>& obstacles,
                                                    const cyber_msgs::LocalizationEstimate& adc_state);
}

#endif