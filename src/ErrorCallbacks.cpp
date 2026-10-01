#include <cave-traversal-tool/ErrorCallbacks.h>

// clang-format off
#include <spdlog/spdlog.h>
// clang-format on

namespace ErrorCallback
{
    void GLFW(const int32_t code, const char* message)
    {
        spdlog::error("GLFW error {} : {}", code, message);
    }
}