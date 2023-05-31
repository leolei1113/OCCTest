#include <vector>
using namespace std;

int maxArea(vector<int>& height)
{
    int i = 0, j = height.size() - 1, area = 0;
    while (i < j)
    {
        int length = j - i;
        if (height[i] < height[j])
        {
            int ht = height[i];
            int newarea = length * ht;
            area = max(area, newarea);
            i++;
        }
        else
        {
            int ht = height[j];
            int newarea = length * ht;
            area = max(area, newarea);
            j--;
        }
        //area = (height[i] < height[j]) ?
            //(max(area, (j - i) * height[i++])) :    //i代表的高度更小，i右移
            //(max(area, (j - i) * height[j--]));  //j代表的高度更小，j左移
    }
    return area;
}

int main()
{
    vector<int> height;
    height.push_back(1);
    height.push_back(8);
    height.push_back(6);
    height.push_back(2);
    height.push_back(5);
    height.push_back(4);
    height.push_back(8);
    height.push_back(3);
    height.push_back(7);

    int MaxArea = maxArea(height);

    return MaxArea;

}