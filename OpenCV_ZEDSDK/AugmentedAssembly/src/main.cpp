// AugmentedAssembly.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "AugmentedAssembly.h"
#include <Windows.h>

int main()
{
    AugmentedAssembly app;
    
    HANDLE hProcess = GetCurrentProcess();
    SetPriorityClass(hProcess, HIGH_PRIORITY_CLASS);
    CloseHandle(hProcess);
    
    try
    {
        app.Start();
    }
    catch(const std::out_of_range& e)
    {
        std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
    }

    return 0;
}
