#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace std;

int main()
{
    // vector<int> temp = {1, 3, 6, 75, 34, 45};
    // for_each(temp.begin(), temp.end(), [](int t) { cout << t << "  "; });
    // int a[10] = {1, 3, 6, 75, 34, 45};
    // int cnt = 1;
    // //值传递
    // for_each(a, a + 6, [cnt](int x) { cout << x + cnt << "  "; });
    // cout << endl;
    // //引用传递
    // for_each(a, a + 6, [&cnt](int x) { cout << x + cnt << "  ";cnt++; });
    // cout << endl;
    auto fun = [](int x) { cout << x << endl; };
    fun(3);
    fun(5);
}