#include "functions.h"

namespace utils {

namespace transforms {

Eigen::Vector2f transformICOM(float x, float y, float theta, float r){
    Eigen::Vector3f input(x, y, 1);
    Eigen::Matrix3f transformation;
    transformation << cos(theta), -sin(theta), r*sin(theta), sin(theta), cos(theta), r-r*cos(theta), 0, 0, 1;
    Eigen::Vector3f output_3;
    output_3 = transformation*input;
    Eigen::Vector2f output;
    output(0) = output_3(0);
    output(1) = output_3(1);
    return output;
}

Eigen::Vector2f projectPoint(Eigen::Vector2f point, float theta, float radius)
{
    Eigen::Matrix3f transformation;
    transformation << cos(theta), -sin(theta), radius * sin(theta), sin(theta), cos(theta), radius - radius * cos(theta), 0, 0, 1;
    Eigen::Vector3f result {transformation * Eigen::Vector3f(point.x(), point.y(), 1)};
    return Eigen::Vector2f(result.x(), result.y());
}

} // namespace transforms

} // namespace utils
