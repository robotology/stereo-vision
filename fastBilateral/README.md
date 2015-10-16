Fast Bilateral Filter implementation for OpenCV

Algorithm and implementation is based on http://people.csail.mit.edu/sparis/bf/
Please cite above paper for research purpose.

### Sample Code

```
#include <iostream>
#include "fastBilateral.hpp"
using namespace std;

int main(int argc, char* argv[])
{
    cv::Mat1b src = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat1b dst;
    cout << "start" << endl;
    const double sigmaColor = 100.0;
    const double sigmaSpace = 10.0;
    cv_extend::bilateralFilter(src, dst, sigmaColor, sigmaSpace);
    cout << "done" << endl;
    cv::imwrite(argv[2], dst);

    return 0;
}

```
