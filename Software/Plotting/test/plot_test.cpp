    
#include <Plotting/matplotlibcpp.h>

namespace plt = matplotlibcpp;

int main() {
    plt::style("seaborn");
    plt::plot({1,3,2,4});
    plt::show();
}