#pragma once
#include <iostream>
#include <string>
#include <privacyconf.h>
//#include <fixed_point.h>

template<typename T>
void printVector(std::string s, T *v, int size) {
    printf("%s\n", s.c_str());
    for (int i = 0; i < size; i++) {
        if (printints)
            printf("%d", *(int*)&v[i]);
        else {
            if (typeid(T) == typeid(float) || typeid(T) == typeid(double)) {
                printf("%.8lf ", v[i]);
            } else {
                printf("%jd ", v[i]);
            }
        }
    }
    printf("\n\n");
}

template<typename T>
void printMatrix(std::string s, T **a, int rows, int cols) {
    printf("%s\n", s.c_str());
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (printints)
                printf("%d", *(int*)&a[i][j]);
            else {
                if (typeid(T) == typeid(float) || typeid(T) == typeid(double)) {
                    printf("%.8lf ", a[i][j]);
                } else {
                    printf("%jd ", a[i][j]);
                }
            }
        }
        printf("\n");
    }
    printf("\n");
}
