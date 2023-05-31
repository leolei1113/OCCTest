
#include <iostream>

using namespace std;

string crackSafe(int n, int k) {
    int kn, kn_1;
    kn = pow(k, n);  //边数
    kn_1 = pow(k, n - 1);   //节点数
    int* num = new int[kn_1];
    fill(num, num + kn_1, k - 1);
    string s(kn + (n - 1), '0');
    for (int i = n - 1, node = 0; i < s.size(); ++i) {
        s[i] = num[node]-- + '0';
        int nMid = s[i - (n - 1)] - '0';
        node = node * k - nMid * kn_1 + num[node] + 1;
    }
    return s;
}

int main()
{
    string result = crackSafe(3, 3);
    return 0;
}


