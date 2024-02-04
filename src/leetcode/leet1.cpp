/*
 * @Author: zhangL2auto
 * @Date: 2024-01-29 15:03:58
 * @LastEditors: 2auto deutschlei47@126.com
 * @LastEditTime: 2024-02-02 10:50:08
 */
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <algorithm>

using namespace std;
bool func(string s1, string s2)
{
    sort(s1.begin(), s1.end());
    sort(s2.begin(), s2.end());

    return s1 == s2;
}

bool func1(string s1, string s2)
{
    if (s1.length() != s2.length()){
        return false;
    }
    vector<int> table(26, 0);
    for (auto i : s1){
        table[i - 'a'] ++;
    }
    for (auto i : s2){
        table[i - 'a'] --;
    }
    for (auto i : table){
        if (i != 0){
            return false;
        }
    }
    return true;
}

int get_sum(int n)
{
    int n_sum = 0;
    while(n){
        n_sum += (n % 10)^2;
        n = n / 10;
    }
    return n_sum;
}

bool func_n(int n)
{
    unordered_set<int> s;
    while(true) {
        int n_sum = get_sum(n);
        n = n_sum;
        if (n_sum == 1){
            return true;
        }
        if (s.find(n_sum) != s.end()){
            return false;
        }
        else{
            s.insert(n_sum);
        }
    }
}

int cal_num(vector<int>& arr1, vector<int>& arr2, vector<int>& arr3, vector<int>& arr4)
{
    int len = arr1.size();
    unordered_map<int, int> m;
    for (auto val1 : arr1){
        for (auto val2 : arr2){
            m[val1 + val2] ++;
        }
    }

    int count = 0;
    for (auto val3 : arr3){
        for (auto val4: arr4){
            if (m.find(0 - val3 - val4) != m.end()){
                count += m[0 - val3 - val4];
            }
        }
    }
    return count;

}


int main()
{
    int a = 6^2;
    cout << a << endl;
}