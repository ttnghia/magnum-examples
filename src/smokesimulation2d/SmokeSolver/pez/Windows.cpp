// Pez was developed by Philip Rideout and released under the MIT License.

#define _WIN32_WINNT 0x0500
#define WINVER       0x0500
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <Pez.h>

#include <Magnum/Magnum.h>
#include <Magnum/GL/GL.h>

namespace Magnum { namespace Examples {
LRESULT WINAPI MsgProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

const char* PezResourcePath() {
    return "demo";
}

#ifdef _MSC_VER

void PezDebugString(const char* pStr, ...) {
    char msg[1024] = { 0 };

    va_list a;
    va_start(a, pStr);

    _vsnprintf_s(msg, _countof(msg), _TRUNCATE, pStr, a);
    OutputDebugStringA(msg);
}

void _PezFatalError(const char* pStr, va_list a) {
    char msg[1024] = { 0 };
    _vsnprintf_s(msg, _countof(msg), _TRUNCATE, pStr, a);
    OutputDebugStringA(msg);
    OutputDebugStringA("\n");
    __debugbreak();
    exit(1);
}

void PezFatalError(const char* pStr, ...) {
    va_list a;
    va_start(a, pStr);
    _PezFatalError(pStr, a);
}

void PezCheckCondition(int condition, ...) {
    va_list     a;
    const char* pStr;

    if(condition) {
        return;
    }

    va_start(a, condition);
    pStr = va_arg(a, const char*);
    _PezFatalError(pStr, a);
}

#else

void PezDebugString(const char* pStr, ...) {
    va_list a;
    va_start(a, pStr);

    char msg[1024] = { 0 };
    vsnprintf(msg, countof(msg), pStr, a);
    fputs(msg, stderr);
}

void _PezFatalError(const char* pStr, va_list a) {
    char msg[1024] = { 0 };
    vsnprintf(msg, countof(msg), pStr, a);
    fputs(msg, stderr);
    __builtin_trap();
    exit(1);
}

void PezFatalError(const char* pStr, ...) {
    va_list a;
    va_start(a, pStr);
    _PezFatalError(pStr, a);
}

void PezCheckCondition(int condition, ...) {
    va_list     a;
    const char* pStr;

    if(condition) {
        return;
    }

    va_start(a, condition);
    pStr = va_arg(a, const char*);
    _PezFatalError(pStr, a);
}

#endif
} }
