#include <rrt-star/rrt-star.h>

#include <fstream>
#include <iostream>

int main(int argc, char const* argv[]) {
    using namespace rrt_star;

    std::vector<ObstacleSPtr> obstacles = {std::make_shared<Circle>(Point(1., 2.), 0.75), std::make_shared<Circle>(Point(3., 0), 0.75),
                                           std::make_shared<Circle>(Point(2., 1.), 0.75), std::make_shared<Circle>(Point(4., -1.), 0.75),
                                           std::make_shared<Circle>(Point(0., 3.), 0.75)};

    RRTStar rrt_s = RRTStar(Point(0, 0), Point(3, 3), obstacles);

    bool successful = rrt_s.compute(300, 0.5);

    if (successful) {
        std::ofstream out("data.json");
        rrt_s.dump(out);
    } else {
        std::cout << "Fail!" << std::endl;
    }

    return 0;
}
