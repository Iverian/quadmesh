#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <fmt/ostream.h>

#include <step_parser.h>

#include <util/itertools.h>

using namespace std;

int main(int argc, char const* argv[])
{

    if (argc > 1) {
        fmt::print(cerr, "filename: {}", argv[1]);

        auto in = ifstream(argv[1]);
        auto manifold = step_parse(in);
        cout << "[" << endl;
        for (auto i = begin(manifold); i != end(manifold); ++i) {
            auto& edges = i->edges();
            for (auto j = begin(edges); j != end(edges); ++j) {
                auto flag
                    = (next(j) == end(edges) && next(i) == end(manifold));
                auto d = j->curve().discretize(j->pfront(), j->pback());
                cout << RangePrint(begin(d), end(d)) << (flag ? "\n" : ",");
            }
        }
        cout << "]" << endl;
    }

    return 0;
}
