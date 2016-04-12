

#include <iostream>
#include "GLWindow.hpp"



int main(int argc, char **argv)
{
    printf("\n\
           -----------------------------------------------------------------------\n\
           OpenGL Sample Program:\n\
           - Drag mouse left-right to rotate camera\n\
           - Hold up-arrow/down-arrow to move camera forward/backward\n\
           - q or ESC to quit\n\
           -----------------------------------------------------------------------\n");

    // general initializations

    //thread();
    GLWindow g;

    boost::thread loop( boost::bind(&GLWindow::thread, &g) );
    while(true) {
        g.updateMainThread();
    }
    
    return 0; // this is just to keep the compiler happy
}