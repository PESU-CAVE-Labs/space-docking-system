#pragma once

#include "Frame.hpp"

class FrameTransformer {
public:
    // Generic: express a state (pos, vel, quat, omega) of frame `target` w.r.t. frame `reference`
    static RelativeState Transform(const ReferenceFrame& target, const ReferenceFrame& reference, const SimState& state);
};
