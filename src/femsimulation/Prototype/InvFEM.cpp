
#include <Magnum/GL/OpenGL.h>

#include <iostream>
#include <algorithm>
#include <time.h>
#include <ctime>
#include <math.h>
#include <fstream>

#include <Common/Setup.h>
#include "FEMSolver/FEMSolver.h"

namespace Magnum { namespace Examples {
/****************************************************************************************************/
static FEMSolver sim;

#define WINDOW_WIDTH          1920
#define WINDOW_HEIGHT         1080

#define DEFAULT_VIEW_ANGLE1   0
#define DEFAULT_VIEW_ANGLE2   0
#define DEFAULT_TRANSLATION_Y 1
#define DEFAULT_TRANSLATION_X -2
#define DEFAULT_TRANSLATION_Z -5

// For 2D
// #define DEFAULT_TRANSLATION_X -5
// #define DEFAULT_TRANSLATION_Z -8

float viewAngle1 = DEFAULT_VIEW_ANGLE1,
      viewAngle2 = DEFAULT_VIEW_ANGLE2,
      viewTransX = DEFAULT_TRANSLATION_X,
      viewTransY = DEFAULT_TRANSLATION_Y,
      viewTransZ = DEFAULT_TRANSLATION_Z;
int mouseMode;
int mouseX, mouseY;

bool forward_sim = false;

enum MouseModes {
    MOUSE_MODE_NONE,
    MOUSE_MODE_VIEW_ROTATE,
    MOUSE_MODE_VIEW_ZOOM,
};

/****************************************************************************************************/
void Update() {
    static constexpr int substeps = 20;
    static int           steps    = 0;
    if(forward_sim) {
        for(int i = 0; i < substeps; ++i) {
            //            sim.step();
        }
        //        std::cout << "steps: " << steps++ << std::endl;
    }
    //    glutPostRedisplay();
}

void Display() {
    //    glClearColor(1.0, 1.0, 1.0, 1.0);
    //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    //    glEnable(GL_DEPTH_TEST);

    //    glPushMatrix();

    //    // Set view
    //    glTranslatef(viewTransX, viewTransY, viewTransZ);
    //    glRotatef(viewAngle1, 1, 0, 0);
    //    glRotatef(viewAngle2, 0, 0, 1);

    //    sim.draw();

    //    glPopMatrix();
    //    glutSwapBuffers();
}

void Reshape(int w, int h) {
    // glViewport(0, 0, w, h);
    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // float r = (float)w / float(h);
    // gluPerspective(60, r, 0.02, 1000.0);
    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
}

void Mouse(int button, int state, int x, int y) {
    mouseX = x;
    mouseY = y;
    //    if(state == GLUT_DOWN) {
    //        switch(button) {
    //            case GLUT_LEFT_BUTTON:
    //                mouseMode = MOUSE_MODE_VIEW_ROTATE;
    //                break;
    //            case GLUT_RIGHT_BUTTON:
    //                mouseMode = MOUSE_MODE_VIEW_ZOOM;
    //                break;
    //        }
    //    } else {
    //        mouseMode = MOUSE_MODE_NONE;
    //    }
    //    glutPostRedisplay();
}

#define VIEW_ROTATE_INC 0.2f
#define VIEW_ZOOM_INC   0.5f

void MouseMove(int x, int y) {
    switch(mouseMode) {
        case MOUSE_MODE_VIEW_ROTATE:
            viewAngle1 -= VIEW_ROTATE_INC * (mouseY - y);
            viewAngle2 -= VIEW_ROTATE_INC * (mouseX - x);
            break;
        case MOUSE_MODE_VIEW_ZOOM:
            viewTransZ += VIEW_ZOOM_INC * (mouseY - y);
            break;
    }

    mouseX = x;
    mouseY = y;

    //    glutPostRedisplay();
}

void GlutKeyboard(unsigned char key, int x, int y) {
    switch(key) {
        case 27: // ESC
            exit(0);
            break;
        case ' ':
            forward_sim ^= true;
            //Debug() << "Simulation:" << (forward_sim ? "Running" : "Paused");
            break;

        case 'c':
        case 'C':
            viewAngle1 = DEFAULT_VIEW_ANGLE1,
            viewAngle2 = DEFAULT_VIEW_ANGLE2,
            viewTransX = DEFAULT_TRANSLATION_X,
            viewTransY = DEFAULT_TRANSLATION_Y,
            viewTransZ = DEFAULT_TRANSLATION_Z;

            //Debug() << "Reset camera";
            break;
        case 'r':
        case 'R':
            //            sim.reset();
            //Debug() << "Reset animation";
            break;
    }
}

//int main(int argc, char** argv) {
//    glutInit(&argc, argv);

//    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
//    if(glutGet(GLUT_SCREEN_WIDTH) > 0 && glutGet(GLUT_SCREEN_HEIGHT) > 0) {
//        glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - WINDOW_WIDTH) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - WINDOW_HEIGHT) / 2);
//    } else { glutInitWindowPosition(50, 50); }
//    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

//    glutCreateWindow("Inv FEM Sim");

//    glutIdleFunc(Update);
//    glutDisplayFunc(Display);
//    glutReshapeFunc(Reshape);
//    glutKeyboardFunc(GlutKeyboard);
//    glutMouseFunc(Mouse);
//    glutMotionFunc(MouseMove);

//    glutMainLoop();

//    return 0;
//}
} }
