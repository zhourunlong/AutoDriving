// Copyright @2018 Pony AI Inc. All rights reserved.
//
#include <bits/stdc++.h>
using namespace std;
int main() {
    long long n;
    scanf("%lld", &n);
    for (int i = 2; i * i <= n; ++i)
        if (!(n % i)) {
            printf("%lld is not a prime!\n", n);
            return 0;
        }
    printf("%lld is a prime!\n", n);
    return 0;
}
