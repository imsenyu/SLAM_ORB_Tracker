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


class Tick {
private:

    time_t mtStart;
    time_t mtEnd;
    double step;
    double fps;
    bool inited;

public:
    Tick(double _fps): fps(_fps), inited(false) {
        fps = fabs(fps);
        if ( fps < 0.01 ) fps = 0.01;
        step = 1.0f/fps;
    }

    bool tick() {
        if ( !inited ) {
            mtStart = clock();
            inited = true;
        }
        else {
            mtEnd = clock();
            double duration = (mtEnd - mtStart) / ((double) CLOCKS_PER_SEC);
            if ( step > duration ) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(  1000*(step - duration)  ));
            }
            mtStart = clock();
        }
        return true;
    }
};

class GLWindow {
private:
    typedef boost::interprocess::interprocess_semaphore semaphore;

    static GLWindow* pThis;
    CGLContextObj obj;
    int argc;
    int mnWindWidth, mnWinHeight;

    int mbSpecialKey;
    int mbIsDragging;
    int meDragMouseButton;

    float mfViewHeight;
    float mfCameraX, mfCameraY, mfCameraZ;
    float mfCameraPosDX, mfCameraPosDY, mfCameraPosDZ;
    float mfCameraDirDX, mfCameraDirDY, mfCameraDirDZ;
    float mfDragX, mfDragY;
    float mfDragDX, mfDragDY;

    float posdx, posdy, posdz;
    float dirdx, dirdy, dirdz;
    float mfCameraPosNX, mfCameraPosNY, mfCameraPosNZ;

    boost::mutex mMutexContext;
    semaphore mSemMain;
public:
    GLWindow(): obj(NULL), argc(0), mnWindWidth(800), mnWinHeight(800),
                mfViewHeight(50),
                posdx(0), posdy(0), posdz(0),
                dirdx(0), dirdy(-1), dirdz(0),

                mbIsDragging(0),
                mSemMain(0),

                mfCameraX(0), mfCameraY(0), mfCameraZ(0),
                mfCameraPosDX(0), mfCameraPosDY(0), mfCameraPosDZ(0),
                mfCameraPosNX(mfCameraX), mfCameraPosNY(mfCameraY), mfCameraPosNZ(mfCameraZ),
                mfCameraDirDX(0), mfCameraDirDY(-1), mfCameraDirDZ(0),
                mfDragX(0), mfDragY(0),
                mfDragDX(0), mfDragDY(0),
                meDragMouseButton(0),

                mbSpecialKey(0)

        {
            pThis = this;
            glutInit(&argc, NULL);
            glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
            glutInitWindowPosition(100, 100);
            glutInitWindowSize(mnWindWidth, mnWinHeight);
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

//    void drawSnowman()
//    {
//        // Draw body (a 20x20 spherical mesh of radius 0.75 at height 0.75)
//        glColor3f(1.0, 1.0, 1.0); // set drawing color to white
//        glPushMatrix();
//        glTranslatef(0.0, 0.0, 0.75);
//        glutSolidSphere(0.75, 20, 20);
//        glPopMatrix();
//
//        // Draw the head (a sphere of radius 0.25 at height 1.75)
//        glPushMatrix();
//        glTranslatef(0.0, 0.0, 1.75); // position head
//        glutSolidSphere(0.25, 20, 20); // head sphere
//
//        // Draw Eyes (two small black spheres)
//        glColor3f(0.0, 0.0, 0.0); // eyes are black
//        glPushMatrix();
//        glTranslatef(0.0, -0.18, 0.10); // lift eyes to final position
//        glPushMatrix();
//        glTranslatef(-0.05, 0.0, 0.0);
//        glutSolidSphere(0.05, 10, 10); // right eye
//        glPopMatrix();
//        glPushMatrix();
//        glTranslatef(+0.05, 0.0, 0.0);
//        glutSolidSphere(0.05, 10, 10); // left eye
//        glPopMatrix();
//        glPopMatrix();
//
//        // Draw Nose (the nose is an orange cone)
//        glColor3f(1.0, 0.5, 0.5); // nose is orange
//        glPushMatrix();
//        glRotatef(90.0, 1.0, 0.0, 0.0); // rotate to point along -y
//        glutSolidCone(0.08, 0.5, 10, 2); // draw cone
//        glPopMatrix();
//        glPopMatrix();
//
//        // Draw a faux shadow beneath snow man (dark green circle)
//        glColor3f(0.0, 0.5, 0.0);
//        glPushMatrix();
//        glTranslatef(0.2, 0.2, 0.001);	// translate to just above ground
//        glScalef(1.0, 1.0, 0.0); // scale sphere into a flat pancake
//        glutSolidSphere(0.75, 20, 20); // shadow same size as body
//        glPopMatrix();
//    }

public:
    void moveCamera() {
        float speed = 0.5f;
        float dx = (mfCameraPosNX - mfCameraX);
        float dy = (mfCameraPosNY - mfCameraY);
        float dz = (mfCameraPosNZ - mfCameraZ);
        float normal = sqrt(dx*dx+dy*dy+dz*dz);
        if (  normal > 1.0f ) {
            dx /= normal;
            dy /= normal;
            dz /= normal;
        }
        mfCameraX = mfCameraX + speed * dx;
        mfCameraY = mfCameraY + speed * dy;
        mfCameraZ = mfCameraZ + speed * dz;
    }
    void setNewCameraPos(float _nx, float _ny, float _nz) {
        mfCameraPosNX = _nx;
        mfCameraPosNY = _ny;
        mfCameraPosNZ = _nz;
    }
    void updateMainThread() {
        bool ret = mSemMain.try_wait();
        if ( ret) {
            boost::mutex::scoped_lock lock(mMutexContext);
            setContext();
            loopOnce();
        }
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

        Tick t(60);
        while(t.tick()) {

            boost::mutex::scoped_lock lock(mMutexContext);
            setContext();
            loopOnce();

        }
        //glutMainLoop();
    }


public:
    virtual void drawAllElement() {
        for(int i=0;i<10;i++) {
            glPushMatrix();
                glTranslatef(i*0.1, 0, i*7.5);
                glColor3f(0.0f, 0.0f, 1.0f); // Let it be blue
                glBegin(GL_QUADS); // 2x2 pixels
                glVertex3f(-1.0f, -1.0f, 0);
                glVertex3f(1.0f, -1.0f, 0);
                glVertex3f(1.0f, 1.0f,0);
                glVertex3f(-1.0f, 1.0f,0);
                glEnd();
            glPopMatrix();
        }
    }

    virtual void onReshape(int nw, int nh)
    {
        mSemMain.post();
        printf("Reshap w %d h %d\n",nw,nh);
        mnWindWidth = nw;
        mnWinHeight = nh;
        float ratio =  ((float) mnWindWidth) / ((float) mnWinHeight); // window aspect ratio
        glMatrixMode(GL_PROJECTION); // projection matrix is active
        glLoadIdentity(); // reset the projection
        gluPerspective(300.0, ratio, 0.01, 1000.0); // perspective transformation
        glMatrixMode(GL_MODELVIEW); // return to modelview mode
        glViewport(0, 0, mnWindWidth, mnWinHeight); // set viewport (drawing area) to entire

    }


    virtual void onIdle(void)
    {
        onDisplay();
        //glutPostRedisplay(); // redisplay everything
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
                    glVertex3f(drawIter * bit,0.0f,-limit);
                    glVertex3f(drawIter * bit,0.0f,limit);
                glEnd();
            }
        }
        for(drawIter = step; drawIter < limit; drawIter += step) {

            for(int bit = -1;bit<=1;bit+=2) {

                glBegin(GL_LINES);
                    glLineWidth(0.5);
                    glVertex3f(-limit,0.0f, drawIter * bit);
                    glVertex3f(limit,0.0f, drawIter * bit);
                glEnd();
            }
        }
        // x-axis
        glColor3f(1.0,0,0);
        glBegin(GL_LINES);
            glLineWidth(5);
            //glVertex3f(0,-limit,0.0f);
            glVertex3f(0,0,0.0f);
            glVertex3f(limit,0,0.0f);
        glEnd();

        // y-axis
        glColor3f(0,1.0,0);
        glBegin(GL_LINES);
            glLineWidth(5);
            //glVertex3f(-limit,0,0.0f);
            glVertex3f(0,0,0.0f);
            glVertex3f(0,limit,0.0f);
        glEnd();

        // z-axis
        glColor3f(0,0,1.0);
        glBegin(GL_LINES);
            glLineWidth(5);
            //glVertex3f(0,0.0f,-limit);
            glVertex3f(0,0.0f,0);
            glVertex3f(0,0.0f,limit);
        glEnd();
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
        //moveCamera();

        gluLookAt(

            mfCameraX + posdx - dirdx * mfViewHeight,
            mfCameraY + posdy + dirdy * mfViewHeight,
            mfCameraZ + posdz + dirdz * mfViewHeight,

            mfCameraX + posdx,
            mfCameraY + posdy ,
            mfCameraZ + posdz,

            0.0,
            0.0,
            -1.0);

        drawCoord();

        drawAllElement();

        glutSwapBuffers(); // Make it all visible
    }

    virtual void onKeyBoard(unsigned char key, int xx, int yy)
    {
        if (key == ESC || key == 'q' || key == 'Q') exit(0);
    }

    virtual void onSpecial(int key, int xx, int yy)
    {
        switch (key) {
            case GLUT_KEY_SHIFT_L :
            case GLUT_KEY_SHIFT_R : mbSpecialKey = 1; break;
        }
    }

    virtual void onSpecialUp(int key, int x, int y)
    {
        switch (key) {
            case GLUT_KEY_SHIFT_L :
            case GLUT_KEY_SHIFT_R : mbSpecialKey = 0; break;
        }
    }

    virtual void onMotion(int x, int y)
    {

        if (mbIsDragging) { // only when dragging
            if ( meDragMouseButton == GLUT_RIGHT_BUTTON ) {

                mfDragDX = (x - mfDragX) * 0.1 ;
                mfDragDY = (y - mfDragY) * 0.1;

                posdx = (mfDragDX + mfCameraPosDX);
                posdy = ( mfCameraPosDY);
                posdz = (mfDragDY + mfCameraPosDZ);


                printf("moveing x %d y %d dargX %f mfDragY %f cameraPosDX %f, camedaDY %f, mfDragDX %f mfDragDY %f\n",x,y, mfDragX, mfDragY, mfCameraPosDX, mfCameraPosDY, mfDragDX, mfDragDY);
            }
            else if ( meDragMouseButton == GLUT_LEFT_BUTTON ) {
                mfDragDX = (x - mfDragX) * 0.01;
                mfDragDY = (y - mfDragY) * 0.01;



                dirdx = (mfDragDX + mfCameraDirDX);
                dirdy = ( mfCameraDirDY);
                dirdz = (mfDragDY + mfCameraDirDZ );

                float normal = sqrt(dirdx*dirdx + dirdy*dirdy+dirdz*dirdz);
                dirdx/= normal;
                dirdy/= normal;
                dirdz/= normal;


                dirdy = fabs(dirdy) < 0.1 ? copysignf(0.1, dirdy) : dirdy;
            }

        }
    }

    virtual void onMouse(int button, int state, int x, int y)
    {
        if ( button == 3 ) {
            printf("3\n");
            mfViewHeight += 0.1;
        }
        else if ( button == 4 ) {
            printf("4\n");
            mfViewHeight -= 0.1;
        }

         if (
            button == GLUT_LEFT_BUTTON ||
            button == GLUT_RIGHT_BUTTON) {


            if (state == GLUT_DOWN && mbIsDragging == 0) { // left mouse button pressed
                meDragMouseButton = button;
                mbIsDragging = 1; // start dragging
                mfDragX = x; // save x where button first pressed
                mfDragY = y; // save x where button first pressed

            }
            else if ( state == GLUT_UP && mbIsDragging == 1) { /* (state = GLUT_UP) */

                mbIsDragging = 0; // no longer dragging
                if ( button == GLUT_RIGHT_BUTTON ) {
                    mfCameraPosDX = posdx; // update camera turning angle
                    mfCameraPosDY = posdy; // update camera turning angle
                    mfCameraPosDZ = posdz; // update camera turning angle
                }
                else if ( button == GLUT_LEFT_BUTTON ) {
                    mfCameraDirDX = dirdx;
                    mfCameraDirDY = dirdy;
                    mfCameraDirDZ = dirdz;
                }
                mfDragDX = 0;
                mfDragDY = 0;

            }

        }
    }



};


#endif /* GLWindow_hpp */
