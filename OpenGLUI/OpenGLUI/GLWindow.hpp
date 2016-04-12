//
//  GLWindow.hpp
//  OpenGLUI
//
//  Created by Sen Yu on 4/13/16.
//  Copyright © 2016 Sen Yu. All rights reserved.
//

#ifndef GLWindow_hpp
#define GLWindow_hpp

#include <cstdio>
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GL/freeglut.h>

#include <boost/thread.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>


#define ESC 27

class GLWindow {
private:
    typedef boost::interprocess::interprocess_semaphore semaphore;

    static GLWindow* pThis;
    CGLContextObj obj;
    int argc;
    int w,h;


    float deltaMove ;
    float lx ;
    float ly ;
    float angle ;
    float deltaAngle;  // incre
    int   isDragging;
    int xDragStart ;    //drag status


    float dh;
    float cameraX, cameraY, cameraZ;
    float cameraDX, cameraDY, cameraDZ;
    float dragX, dragY;
    float dragDX, dragDY;

    boost::mutex mMutexContext;
    semaphore mSemMain;
public:
    GLWindow(): obj(NULL), argc(0), w(800), h(800),
               lx(0),ly(0),

                isDragging(0),
                mSemMain(0),


                dh(0),
                cameraX(0), cameraY(0), cameraZ(20),
                cameraDX(0), cameraDY(0), cameraDZ(0),
                dragX(0), dragY(0),
                dragDX(0), dragDY(0)
        {
            pThis = this;
            glutInit(&argc, NULL);
            glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
            glutInitWindowPosition(100, 100);
            glutInitWindowSize(w, h);
            glutCreateWindow("OpenGL/GLUT Sampe Program");

            glutReshapeFunc(_funcReshap); // window reshape callback
            glutDisplayFunc(_funcDisplay); // (re)display callback
            glutIdleFunc(_funcIdle); // incremental update
            glutIgnoreKeyRepeat(1); // ignore key repeat when holding key down
            glutMouseFunc(_funcMouse); // process mouse button push/release
            glutMotionFunc(_funcMotion); // process mouse dragging motion
            glutKeyboardFunc(_funcKeyboard); // process standard key clicks
            glutSpecialFunc(_funcSpecial); // process special key pressed
            // Warning: Nonstandard function! Delete if desired.
            glutSpecialUpFunc(_funcSpecialUp); // process special key release

            // OpenGL init
            glEnable(GL_DEPTH_TEST);
            obj = CGLGetCurrentContext();



        }
private:
    static void _funcReshap(int x, int y) {
        pThis->onReshape(x,y);
    }
    static void _funcDisplay() {
        pThis->onDisplay();
    }
    static void _funcIdle() {
        pThis->onIdle();
    }
    static void _funcMouse(int button, int state, int x, int y) {
        pThis->onMouse(button, state, x, y);
    }
    static void _funcMotion(int x, int y) {
        pThis->onMotion(x,y);
    }
    static void _funcKeyboard(unsigned char key, int xx, int yy) {
        pThis->onKeyBoard(key,xx,yy);
    }
    static void _funcSpecial(int key, int xx, int yy) {
        pThis->onSpecial(key,xx,yy);
    }
    static void _funcSpecialUp(int key, int xx, int yy) {
        pThis->onSpecialUp(key,xx,yy);
    }



public:
    void updateMainThread() {
        bool ret = mSemMain.try_wait();
        if ( ret) {
            boost::mutex::scoped_lock lock(mMutexContext);
            setContext();
            loopOnce();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    void setContext() {
        int e = CGLSetCurrentContext(obj);
        printf("setContext e %d\n",e);
    }
    void loopOnce() {
        glutPostRedisplay();
        glutMainLoopEvent();
    }
    void thread() {


        while(true) {
            boost::mutex::scoped_lock lock(mMutexContext);
            setContext();
            loopOnce();

        }
        //glutMainLoop();
    }


public:
    virtual void onReshape(int nw, int nh)
    {
        mSemMain.post();
        printf("Reshap w %d h %d\n",nw,nh);
        w = nw;
        h = nh;
        float ratio =  ((float) w) / ((float) h); // window aspect ratio
        glMatrixMode(GL_PROJECTION); // projection matrix is active
        glLoadIdentity(); // reset the projection
        gluPerspective(300.0, ratio, 0.01, 1000.0); // perspective transformation
        glMatrixMode(GL_MODELVIEW); // return to modelview mode
        glViewport(0, 0, w, h); // set viewport (drawing area) to entire

    }

    static void drawSnowman()
    {
        // Draw body (a 20x20 spherical mesh of radius 0.75 at height 0.75)
        glColor3f(1.0, 1.0, 1.0); // set drawing color to white
        glPushMatrix();
        glTranslatef(0.0, 0.0, 0.75);
        glutSolidSphere(0.75, 20, 20);
        glPopMatrix();

        // Draw the head (a sphere of radius 0.25 at height 1.75)
        glPushMatrix();
        glTranslatef(0.0, 0.0, 1.75); // position head
        glutSolidSphere(0.25, 20, 20); // head sphere

        // Draw Eyes (two small black spheres)
        glColor3f(0.0, 0.0, 0.0); // eyes are black
        glPushMatrix();
        glTranslatef(0.0, -0.18, 0.10); // lift eyes to final position
        glPushMatrix();
                    glTranslatef(-0.05, 0.0, 0.0);
                    glutSolidSphere(0.05, 10, 10); // right eye
        glPopMatrix();
        glPushMatrix();
                    glTranslatef(+0.05, 0.0, 0.0);
                    glutSolidSphere(0.05, 10, 10); // left eye
        glPopMatrix();
        glPopMatrix();

        // Draw Nose (the nose is an orange cone)
        glColor3f(1.0, 0.5, 0.5); // nose is orange
        glPushMatrix();
        glRotatef(90.0, 1.0, 0.0, 0.0); // rotate to point along -y
        glutSolidCone(0.08, 0.5, 10, 2); // draw cone
        glPopMatrix();
        glPopMatrix();

        // Draw a faux shadow beneath snow man (dark green circle)
        glColor3f(0.0, 0.5, 0.0);
        glPushMatrix();
        glTranslatef(0.2, 0.2, 0.001);	// translate to just above ground
        glScalef(1.0, 1.0, 0.0); // scale sphere into a flat pancake
        glutSolidSphere(0.75, 20, 20); // shadow same size as body
        glPopMatrix();
    }

    virtual void onIdle(void)
    {
//        if (deltaMove) { // update camera position
//            x += deltaMove * lx * 0.1;
//            y += deltaMove * ly * 0.1;
//        }
        glutPostRedisplay(); // redisplay everything
    }

    void drawCoord(int step = 10, float limit = 1e3) {
        float drawIter;
        glColor3f(0.2,0.2,0.2);
        glEnable (GL_LINE_SMOOTH);

        glEnable (GL_BLEND);//启用混合
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        /*对图像质量和渲染速度之间控制权衡关系*/

        for(drawIter = step; drawIter < limit; drawIter += step) {

            for(int bit = -1;bit<=1;bit+=2) {

                glBegin(GL_LINES);
                glLineWidth(0.5);
                glVertex3f(drawIter * bit,-limit,0.0f);
                glVertex3f(drawIter * bit,limit,0.0f);
                glEnd();
            }
        }
        for(drawIter = step; drawIter < limit; drawIter += step) {

            for(int bit = -1;bit<=1;bit+=2) {

                glBegin(GL_LINES);
                glLineWidth(0.5);
                glVertex3f(-limit, drawIter * bit,0.0f);
                glVertex3f(limit, drawIter * bit,0.0f);
                glEnd();
            }
        }
    }
    virtual void onDisplay(void)
    {
        int i, j;

        // Clear color and depth buffers
        glClearColor(0.0, 0.7, 1.0, 1.0); // sky color is light blue
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Reset transformations
        glLoadIdentity();

        // Set the camera centered at (x,y,1) and looking along directional
        // vector (lx, ly, 0), with the z-axis pointing up
        float lx = (dragDX+cameraDX);
        float ly = (dragDY+cameraDY);


        gluLookAt(
                  cameraX + lx,      cameraY - ly,      cameraZ + dh,
            cameraX + lx, cameraY - cameraZ/2 - ly , 0,
                  0.0,    1.0,    0.0);

        drawCoord();
        // Draw ground - 200x200 square colored green
//        glColor3f(0.0, 0.0, 0.0);
//        glBegin(GL_LINES);
//        glLineWidth(1);
//        glVertex3f(-100.0f,-100.0f,0.0f);
//        glVertex3f(100.0f,-100.0f,0.0f);
//        glEnd();

        // Draw 36 snow men
        for(i = -3; i < 3; i++)
            for(j = -3; j < 3; j++) {
                glPushMatrix();
                glTranslatef(i*7.5, j*7.5, 0);
                drawSnowman();
                glPopMatrix();
            }

        glutSwapBuffers(); // Make it all visible
    }

    virtual void onKeyBoard(unsigned char key, int xx, int yy)
    {
        if (key == ESC || key == 'q' || key == 'Q') exit(0);
    }

    virtual void onSpecial(int key, int xx, int yy)
    {
//        switch (key) {
//            case GLUT_KEY_UP : deltaMove = 1.0; break;
//            case GLUT_KEY_DOWN : deltaMove = -1.0; break;
//        }
    }

    virtual void onSpecialUp(int key, int x, int y)
    {
//        switch (key) {
//            case GLUT_KEY_UP : deltaMove = 0.0; break;
//            case GLUT_KEY_DOWN : deltaMove = 0.0; break;
//        }
    }

    virtual void onMotion(int x, int y)
    {

        if (isDragging) { // only when dragging

            dragDX = (x - dragX) * 0.1 ;
            dragDY = (y - dragY) * 0.1;
            printf("moveing x %d y %d dargX %f dragY %f cameraDX %f, camedaDY %f, dragDX %f dragDY %f\n",x,y,dragX, dragY, cameraDX, cameraDY, dragDX, dragDY);

        }
    }

    virtual void onMouse(int button, int state, int x, int y)
    {
        if ( button == 3 ) {
            printf("3\n");
            dh += 0.1;
        }
        else if ( button == 4 ) {
            printf("4\n");
            dh -= 0.1;
        }

        if (button == GLUT_LEFT_BUTTON) {

            if (state == GLUT_DOWN) { // left mouse button pressed
                isDragging = 1; // start dragging
                dragX = x; // save x where button first pressed
                dragY = y; // save x where button first pressed

            }
            else if ( state == GLUT_UP ) { /* (state = GLUT_UP) */

                isDragging = 0; // no longer dragging
                cameraDX += dragDX; // update camera turning angle
                cameraDY += dragDY; // update camera turning angle
                dragDX = 0;
                dragDY = 0;

            }

        }
    }



};


#endif /* GLWindow_hpp */
