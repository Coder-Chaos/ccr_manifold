#pragma once 
#include <functional>

namespace common {

    // accelerator function pointer
    template<typename T>
    struct Accelerator
    {
        // init accelerator
        std::function<int(T*, const char*)> Initialize;

        // release accelerator
        std::function<int(T*)> Release;

        // start
        std::function<void(T*)> Start;

        // done
        std::function<uint32_t(T*)> IsDone;

        // Set parameter one
        std::function<void(T*, u64)> SetParamOne;

        // Set parameter two
        std::function<void(T*, u64)> SetParamTwo;

        // Set parameter three
        std::function<void(T*, u64)> SetParamThree;

        // Set parameter four
        std::function<void(T*, u64)> SetParamFour;
    };

}