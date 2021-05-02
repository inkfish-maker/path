#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;

int main()
{
    // pair<int, int> p1(1, 4);
    // pair<int, int> p2(2, 5);
    // pair<int, int> p3(3, 6);
    // pair<int, int> p4(4, 7);
    // pair<int, int> p5(5, 8);
    // vector<pair<int, int>> ps = {p1, p2, p3, p4, p5};
    // vector<pair<int, int>>::iterator i = find(ps.begin(), ps.end(), [](pair<int, int> p) { return p.first == 2; });
    // cout << i->first << "  " << i->second << endl;
    vector<int> vec = {1, 33, 345, 234, 24, 6465, 234};
    cout << vec.size() << "," << vec.capacity() << endl;
    vec.resize(0);
    cout << vec.size() << "," << vec.capacity() << endl;
}