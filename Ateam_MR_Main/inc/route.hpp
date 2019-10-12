/*
 * route.hpp
 *
 *  Created on: 2019/09/01
 *      Author: User
 */

#ifndef ROUTE_HPP_
#define ROUTE_HPP_

#include "PathPoint.hpp"
#include <vector>
#include <array>

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

using Tpos = float;
using Tvel = uint32_t;

extern const std::array<std::vector<pathPoint<float>>, 5> running_path;

#endif /* ROUTE_HPP_ */
