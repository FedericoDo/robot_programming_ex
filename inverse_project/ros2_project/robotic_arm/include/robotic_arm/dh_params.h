#include <vector>
#include <string>

struct DHParams {
    double theta;  // Joint angle (in radians)
    double d;     // Link offset
    double a;     // Link length
    double alpha; // Link twist (in radians)

    static std::vector<DHParams> loadParams(const std::string& filepath);

    std::string toString() const;
};