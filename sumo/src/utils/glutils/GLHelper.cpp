/****************************************************************************/
/// @file    GLHelper.cpp
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id$
///
// Some methods which help to draw certain geometrical objects in openGL
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// copyright : (C) 2001-2007
//  by DLR (http://www.dlr.de/) and ZAIK (http://www.zaik.uni-koeln.de/AFS)
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
// ===========================================================================
// compiler pragmas
// ===========================================================================
#ifdef _MSC_VER
#pragma warning(disable: 4786)
#endif


// ===========================================================================
// included modules
// ===========================================================================
#ifdef WIN32
#include <windows_config.h>
#else
#include <config.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>

#include "GLHelper.h"
#include <utils/geom/GeomHelper.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// static member definitions
// ===========================================================================
std::vector<std::pair<SUMOReal, SUMOReal> > GLHelper::myCircleCoords;
int GLHelper::myStippleTexture = 1;


// ===========================================================================
// method definitions
// ===========================================================================
void
GLHelper::drawFilledPoly(const Position2DVector &v, bool close)
{
    if (v.size()==0) {
        return;
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    const Position2DVector::ContType &l = v.getCont();
    for (Position2DVector::ContType ::const_iterator i=l.begin(); i!=l.end(); i++) {
        const Position2D &p = *i;
        glVertex2d(p.x(), p.y());
    }
    if (close) {
        const Position2D &p = *(l.begin());
        glVertex2d(p.x(), p.y());
    }
    glEnd();
}


void
GLHelper::drawBoxLine(const Position2D &beg, SUMOReal rot, SUMOReal visLength,
                      SUMOReal width)
{
    glPushMatrix();
    glTranslated(beg.x(), beg.y(), 0);
    glRotated(rot, 0, 0, 1);
    glBegin(GL_QUADS);
    glVertex2d(-width, 0);
    glVertex2d(-width, -visLength);
    glVertex2d(width, -visLength);
    glVertex2d(width, 0);
    glEnd();
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2d(0, -visLength);
    glEnd();
    glPopMatrix();
}

void
GLHelper::drawBoxLine(const Position2D &beg1, const Position2D &beg2,
                      SUMOReal rot, SUMOReal visLength,
                      SUMOReal width)
{
    glPushMatrix();
    glTranslated((beg2.x()+beg1.x())*.5, (beg2.y()+beg1.y())*.5, 0);
//    glTranslated(beg1.x(), beg1.y(), 0);
    glRotated(rot, 0, 0, 1);
    glBegin(GL_QUADS);
    glVertex2d(-width, 0);
    glVertex2d(-width, -visLength);
    glVertex2d(width, -visLength);
    glVertex2d(width, 0);
    glEnd();
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2d(0, -visLength);
    glEnd();
    glPopMatrix();
}

/*
void
GLHelper::drawBoxLine(const Position2D &beg1, const Position2D &beg2,
                      SUMOReal rot, SUMOReal visLength)
{
    SUMOReal width = GeomHelper::distance(beg1, beg2);
    glPushMatrix();
    glTranslated(beg1.x(), beg1.y(), 0);
    glRotated( rot, 0, 0, 1 );
    glBegin(GL_QUADS);
    glVertex2d(-width, 0);
    glVertex2d(-width, -visLength);
    glVertex2d(width, -visLength);
    glVertex2d(width, 0);
    glEnd();
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2d(0, -visLength);
    glEnd();
    glPopMatrix();
}
*/


void
GLHelper::drawBoxLines(const Position2DVector &geom,
                       const std::vector<SUMOReal> &rots,
                       const std::vector<SUMOReal> &lengths,
                       SUMOReal width)
{
    for (size_t i=0; i<geom.size()-1; i++) {
        drawBoxLine(geom[i], rots[i], lengths[i], width);
    }
}


void
GLHelper::drawBoxLines(const Position2DVector &geom1,
                       const Position2DVector &geom2,
                       const std::vector<SUMOReal> &rots,
                       const std::vector<SUMOReal> &lengths,
                       SUMOReal width)
{
    for (size_t i=0; i<geom1.size()-1; i++) {
        GLHelper::drawBoxLine(geom1[i], geom2[i], rots[i], lengths[i], width);
    }
}


void
GLHelper::drawStippleBoxLines(const Position2DVector &geom1,
                              const std::vector<SUMOReal> &rots,
                              const std::vector<SUMOReal> &lengths,
                              SUMOReal width)
{
    /*
    if(myStippleTexture<0) {
        FXImage *img = MFXImageHelper::loadimage(getApp(), "D:\\bild.bmp");
        myStippleTexture = GUITexturesHelper::add(img);
    }
    for(size_t i=0; i<geom1.size()-1; i++) {
        glPushMatrix();
        glTranslated(beg.x(), beg.y(), 0);
        glRotated( rot, 0, 0, 1 );
        GUITexturesHelper::drawTexturedBox(myStippleTexture, -.75, 0, .75, lengths[i]);
        glPopMatrix();
    }
    */
}


void
GLHelper::drawBoxLines(const Position2DVector &geom, SUMOReal width)
{
    for (size_t i=0; i<geom.size()-1; i++) {
        const Position2D &f = geom[i];
        const Position2D &s = geom[i+1];
        drawBoxLine(f,
                    (SUMOReal) atan2((s.x()-f.x()), (f.y()-s.y()))*(SUMOReal) 180.0/(SUMOReal) 3.14159265,
                    GeomHelper::distance(f, s),
                    width);
    }
}


void
GLHelper::drawLine(const Position2D &beg, SUMOReal rot, SUMOReal visLength)
{
    glPushMatrix();
    glTranslated(beg.x(), beg.y(), 0);
    glRotated(rot, 0, 0, 1);
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2d(0, -visLength);
    glEnd();
    glPopMatrix();
}


void
GLHelper::drawLine(const Position2D &beg1, const Position2D &beg2,
                   SUMOReal rot, SUMOReal visLength)
{
    glPushMatrix();
    glTranslated((beg2.x()+beg1.x())*.5, (beg2.y()+beg1.y())*.5, 0);
    glRotated(rot, 0, 0, 1);
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2d(0, -visLength);
    glEnd();
    glPopMatrix();
}



void
GLHelper::drawFilledCircle(SUMOReal width, int steps)
{
    drawFilledCircle(width, steps, 0, 360);
}


void
GLHelper::drawFilledCircle(SUMOReal width, int steps, SUMOReal beg, SUMOReal end)
{
    if (myCircleCoords.size()==0) {
        for (int i=0; i<360; i+=10) {
            SUMOReal x = (SUMOReal) sin((SUMOReal) i / 180.0 * PI);
            SUMOReal y = (SUMOReal) cos((SUMOReal) i / 180.0 * PI);
            myCircleCoords.push_back(std::pair<SUMOReal, SUMOReal>(x, y));
        }
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    std::pair<SUMOReal, SUMOReal> p1 =
        beg==0 ? myCircleCoords[0] : myCircleCoords[((int) beg/10)%36];
    for (int i=(int)(beg/10); i<steps&&(36.0/(SUMOReal) steps *(SUMOReal) i)*10<end; i++) {
        const std::pair<SUMOReal, SUMOReal> &p2 =
            myCircleCoords[(size_t)(36.0/(SUMOReal) steps * (SUMOReal) i)];
        glBegin(GL_TRIANGLES);
        glVertex2d(p1.first * width, p1.second * width);
        glVertex2d(p2.first * width, p2.second * width);
        glVertex2d(0, 0);
        glEnd();
        p1 = p2;
    }
    const std::pair<SUMOReal, SUMOReal> &p2 =
        end==360 ? myCircleCoords[0] : myCircleCoords[((int) end/10)%36];
    glBegin(GL_TRIANGLES);
    glVertex2d(p1.first * width, p1.second * width);
    glVertex2d(p2.first * width, p2.second * width);
    glVertex2d(0, 0);
    glEnd();
}


void
GLHelper::drawOutlineCircle(SUMOReal width, SUMOReal iwidth, int steps)
{
    drawOutlineCircle(width, iwidth, steps, 0, 360);
}


void
GLHelper::drawOutlineCircle(SUMOReal width, SUMOReal iwidth, int steps, SUMOReal beg, SUMOReal end)
{
    if (myCircleCoords.size()==0) {
        for (int i=0; i<360; i+=10) {
            SUMOReal x = (SUMOReal) sin((SUMOReal) i / 180.0 * PI);
            SUMOReal y = (SUMOReal) cos((SUMOReal) i / 180.0 * PI);
            myCircleCoords.push_back(std::pair<SUMOReal, SUMOReal>(x, y));
        }
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    std::pair<SUMOReal, SUMOReal> p1 =
        beg==0 ? myCircleCoords[0] : myCircleCoords[((int) beg/10)%36];
    for (int i=(int)(beg/10); i<steps&&(36.0/(SUMOReal) steps *(SUMOReal) i)*10<end; i++) {
        const std::pair<SUMOReal, SUMOReal> &p2 =
            myCircleCoords[(size_t)(36.0/(SUMOReal) steps * (SUMOReal) i)];
        glBegin(GL_TRIANGLES);
        glVertex2d(p1.first * width, p1.second * width);
        glVertex2d(p2.first * width, p2.second * width);
        glVertex2d(p2.first * iwidth, p2.second * iwidth);

        glVertex2d(p2.first * iwidth, p2.second * iwidth);
        glVertex2d(p1.first * iwidth, p1.second * iwidth);
        glVertex2d(p1.first * width, p1.second * width);
        glEnd();
        p1 = p2;
    }
    const std::pair<SUMOReal, SUMOReal> &p2 =
        end==360 ? myCircleCoords[0] : myCircleCoords[((int) end/10)%36];
    glBegin(GL_TRIANGLES);
    glVertex2d(p1.first * width, p1.second * width);
    glVertex2d(p2.first * width, p2.second * width);
    glVertex2d(p2.first * iwidth, p2.second * iwidth);

    glVertex2d(p2.first * iwidth, p2.second * iwidth);
    glVertex2d(p1.first * iwidth, p1.second * iwidth);
    glVertex2d(p1.first * width, p1.second * width);
    glEnd();
}


void
GLHelper::drawTriangleAtEnd(const Line2D &l, float tLength,
                            float tWidth)
{
    if (l.length()<tLength) {
        tWidth = tWidth * l.length() / tLength;
        tLength = l.length();
    }
    Line2D rl(l.getPositionAtDistance(l.length()-tLength), l.p2());
    glPushMatrix();
    glTranslated(rl.p1().x(), rl.p1().y(), 0);
    glRotated(-l.atan2DegreeAngle(), 0, 0, 1);
    glBegin(GL_TRIANGLES);
    glVertex2d(0, -tLength);
    glVertex2d(-tWidth, 0);
    glVertex2d(+tWidth, 0);
    glEnd();
    glPopMatrix();
}



/****************************************************************************/

