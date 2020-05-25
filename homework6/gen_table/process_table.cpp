#include <bits/stdc++.h>
using namespace std;
int main() {
    freopen("brake.txt", "r", stdin);
    freopen("temp.txt", "w", stdout);
    double x, y;
    printf("0,");
    while (~scanf("%lf%lf", &x, &y)) {
        printf("%.5lf,", y);
    }
    return 0;
}
