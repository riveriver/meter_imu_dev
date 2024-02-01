#include <iostream>
#include <vector>
template<typename T>
class SlidingWindowFilter {
private:
    std::vector<T> window;  // 滑动窗口
    int windowSize;        // 窗口大小

public:
    SlidingWindowFilter(int size = 200) : windowSize(size) {}
    // 添加新的测量值到窗口，并返回滤波结果
    T filterEigen(T newValue) {
        window.push_back(newValue);  // 将新值加入窗口

        // 如果窗口大小超过设定值，移除最早的值
        if (window.size() > windowSize) {
            window.erase(window.begin());
        }

        // 计算窗口内的平均值作为滤波结果
        T sum = T::Zero();
        for (const T& value : window) {
            sum += value;
        }
        return sum / window.size();
    }

    T filter(T newValue) {
        window.push_back(newValue);  // 将新值加入窗口

        // 如果窗口大小超过设定值，移除最早的值
        if (window.size() > windowSize) {
            window.erase(window.begin());
        }

        // 计算窗口内的平均值作为滤波结果
        T sum = 0;
        for (const T& value : window) {
            sum += value;
        }
        return sum / window.size();
    }
};