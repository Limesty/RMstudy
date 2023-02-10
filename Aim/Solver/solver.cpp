#include "solver.hpp"
#include <cmath>
Solver::Solver(){};
Solver::~Solver(){};
Point2f Solver::solve(Point2f LeftUpPoint, Point2f RightDownPoint, Size size) {
    Point2f centerPoint = (LeftUpPoint + RightDownPoint) / 2;
    Rect2f rectObj(LeftUpPoint, RightDownPoint);
    float x_offset = 1.0f - 2 * centerPoint.x / size.width, y_offset = 1.0f - 2 * centerPoint.y / size.height;
    float yaw = atan(x_offset * YAW_HALF_PERSPECT);
    float pitch = atan(y_offset * PTICH_HALF_PERSPECT);
    float dis = size.area() / rectObj.area();
    return Point2f(yaw, pitch);
}