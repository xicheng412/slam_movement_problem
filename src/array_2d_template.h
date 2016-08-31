#ifndef ARRAY_2D_TEMPLATE_H
#define ARRAY_2D_TEMPLATE_H

#include <cstdlib>
using namespace std;

//using template,so all code must be write in h file.

//malloc 2d array
template <typename T>  
T** new_Array2D(int row, int col)  
{  
    int size = sizeof(T);  
    int point_size = sizeof(T*);  
    T **arr = (T **) malloc(point_size * row + size * row * col);  
    if (arr != NULL)  
    {     
        T *head = (T*)((int)arr + point_size * row);  
        for (int i = 0; i < row; ++i)  
        {  
            arr[i] =  (T*)((int)head + i * col * size);  
            for (int j = 0; j < col; ++j)  
                new (&arr[i][j]) T;  
        }  
    }  
    return (T**)arr;  
} 

//release 2d array 
template <typename T>  
void delete_Array2D(T **arr, int row, int col)  
{  
    for (int i = 0; i < row; ++i)  
        for (int j = 0; j < col; ++j)  
            arr[i][j].~T();  
    if (arr != NULL)  
        free((void**)arr);  
}  

#endif
