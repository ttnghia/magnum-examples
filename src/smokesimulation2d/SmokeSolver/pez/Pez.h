#pragma once
#include <Magnum/GL/OpenGL.h>
using namespace Magnum;

namespace Magnum { namespace Examples {
// Implemented by the platform layer:
const char* PezResourcePath();
void        PezDebugString(const char* pStr, ...);
void        PezDebugStringW(const wchar_t* pStr, ...);
void        PezFatalError(const char* pStr, ...);
void        PezFatalErrorW(const wchar_t* pStr, ...);
void        PezCheckCondition(int condition, ...);
void        PezCheckConditionW(int condition, ...);
int         PezIsPressing(char key);

// Configuration:
#define PEZ_VIEWPORT_WIDTH        (800)
#define PEZ_VIEWPORT_HEIGHT       (600)
#define PEZ_ENABLE_MULTISAMPLING  0
#define PEZ_VERTICAL_SYNC         1
#define PEZ_FORWARD_COMPATIBLE_GL 0
} }
