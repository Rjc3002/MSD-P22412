#include "StewartPlatform.h"

class StewartPlatform {
private:
    std::array<double, 9> design_variable;
    InvKinematics clf;
    std::array<double, 6> l;

public:
    StewartPlatform(std::array<double, 9> dv) : design_variable(dv), clf(dv[0], dv[1], dv[2], dv[3], dv[4], dv[5], dv[6], dv[7], dv[8]) {}

    void initStewart() {
        std::array<double, 3> translation = { 0, 0, 0 };
        std::array<double, 3> rotation = { 0, 0, 0 };
        l = clf.solve(translation, rotation);
        std::cout << "Initial leg length (m): ";
        for (const auto& length : l) std::cout << length << " ";
        std::cout << std::endl;
    }

    bool searchSimulation(std::array<double, 3> trans, std::array<double, 3> rot) {
        std::cout << "Starting search" << std::endl;
        bool found = false;
        std::vector<std::array<double, 4>> results;

        std::vector<double> rvalues = { 0.05, 0.04, 0.03, 0.02, 0.01 };
        std::vector<double> gvalues = { 60, 50, 40, 30, 20, 10 };

        for (double r1 : rvalues) {
            for (double r2 : rvalues) {
                for (double g1 : gvalues) {
                    for (double g2 : gvalues) {
                        clf = InvKinematics(r1, r2, g1 / 2.0, g2 / 2.0, true, design_variable[5], design_variable[6], design_variable[7], design_variable[8]);
                        if (clf.solve(trans, rot, true)) {
                            results.push_back({ r1, r2, g1, g2 });
                            found = true;
                        }
                    }
                }
            }
        }

        if (found) {
            std::cout << "Search Results: \n   rB    rp    gB     gP" << std::endl;
            for (const auto& res : results) {
                std::cout << res[0] << " " << res[1] << " " << res[2] << " " << res[3] << std::endl;
            }
            return true;
        }
        return false;
    }

    void startSimulation(std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> data, bool searching = false) {
        std::cout << "Starting Simulation" << std::endl;
        initStewart();

        for (const auto& d : data) {
            bool test = false;
            std::array<double, 3> trans = d.first;
            std::array<double, 3> rot = d.second;

            if (searching) {
                test = searchSimulation(trans, rot);
            }

            if (!test && searching) {
                std::cout << "Search Failed..." << std::endl;
                return;
            }
            else if (test && searching) {
                std::cout << "Search Successful" << std::endl;
            }

            std::array<double, 6> new_l = clf.solve(trans, rot);

            std::cout << "Leg actuation distance (m): ";
            for (size_t i = 0; i < 6; ++i) {
                std::cout << (new_l[i] - l[i]) << " ";
            }
            std::cout << std::endl;
        }
    }
};

int main() {
    std::array<double, 9> dv = { 0.1, 0.05, 30, 30, true, 0.1, 0, 0, 0 };
    StewartPlatform sp(dv);
    sp.initStewart();
    return 0;
}
