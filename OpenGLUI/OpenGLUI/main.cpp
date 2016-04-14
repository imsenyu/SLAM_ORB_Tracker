

#include <iostream>
#include "GLWindow.hpp"∂



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
   // cv::namedWindow("gl", CV_WINDOW_OPENGL);
   // cv::setOpenGlContext("gl");
    //thread();
    cv::namedWindow("m", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    cv::waitKey(10);
    GLWindow g;
    Tick t(10);

    cv::Mat im(100,100, CV_8U);
    im = true ? 255 : 0;
    bool b = false;


    while( t.tick() ) {
      //  cv::setOpenGlContext("gl");


        b = !b;
        im = b ? 255:0;
        cv::imshow("m",im);
        cv::waitKey(10);


        g.loopOnce();
    }
    
    return 0; // this is just to keep the compiler happy
}