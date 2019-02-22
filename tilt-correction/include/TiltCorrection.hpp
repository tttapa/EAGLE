#pragma once

#include <Quaternion.hpp>

/// @see    eagle-control-slides.pdf pp.178-181
ColVector<3> getCorrectedPosition(ColVector<2> rawPosition, double rawHeight,
                                  Quaternion orientation);