/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    GNEFrame.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2016
/// @version $Id$
///
// The Widget for add additional elements
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <utils/foxtools/fxexdefs.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIIOGlobals.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/images/GUIIconSubSys.h>

#include "GNEFrame.h"
#include "GNEViewParent.h"
#include "GNEViewNet.h"
#include "GNEAttributeCarrier.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEFrame::GEOAttributes) GNEFrameGEOAttributes[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNEFRAME_LONGITUDE,     GNEFrame::GEOAttributes::onCmdSetLongitude),
    FXMAPFUNC(SEL_COMMAND,  MID_GNEFRAME_LATITUDE,      GNEFrame::GEOAttributes::onCmdSetLatitude),
    FXMAPFUNC(SEL_COMMAND,  MID_GNEFRAME_USEGEO,        GNEFrame::GEOAttributes::onCmdUseGEOParameters),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,                   GNEFrame::GEOAttributes::onCmdHelp),
};

FXIMPLEMENT(GNEFrame::GEOAttributes,    FXGroupBox,     GNEFrameGEOAttributes,  ARRAYNUMBER(GNEFrameGEOAttributes))


// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNEFrame::GEOAttributes - methods
// ---------------------------------------------------------------------------

GNEFrame::GEOAttributes::GEOAttributes(FXComposite* parent):
    FXGroupBox(parent, "GEO Attributes", GUIDesignGroupBoxFrame),
    myAc(NULL) {

    // Create Frame for longitude
    FXHorizontalFrame* longitudeFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myLongitudeLabel = new FXLabel(longitudeFrame, toString(SUMO_ATTR_LENGTH).c_str(), 0, GUIDesignLabelAttribute);
    myLongitudeTextField = new FXTextField(longitudeFrame, GUIDesignTextFieldNCol, this, MID_GNEFRAME_LONGITUDE, GUIDesignTextField);

    // Create Frame for latitude
    FXHorizontalFrame* latitudeFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myLatitudeLabel = new FXLabel(latitudeFrame, toString(SUMO_ATTR_LENGTH).c_str(), 0, GUIDesignLabelAttribute);
    myLatitudeTextField = new FXTextField(latitudeFrame, GUIDesignTextFieldNCol, this, MID_GNEFRAME_LATITUDE, GUIDesignTextField);
    
    // Create Frame for use GEO
    FXHorizontalFrame* useGEOFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);
    myUseGEOLabel = new FXLabel(useGEOFrame, "Use GEO attributes", 0, GUIDesignLabelAttribute);
    myUseGEOCheckButton = new FXCheckButton(useGEOFrame, "false", this, MID_GNEFRAME_USEGEO, GUIDesignCheckButtonAttribute);
    
    // Create help button
    myHelpButton = new FXButton(this, "Help", 0, this, MID_HELP, GUIDesignButtonRectangular);
}


GNEFrame::GEOAttributes::~GEOAttributes() {}


void 
GNEFrame::GEOAttributes::showGEOAttributes(GNEAttributeCarrier *ac) {
    myAc = ac;
    // fill attributes
    UpdateGEOAttributes();
    // show FXGroupBox
    FXGroupBox::show();
}


void 
GNEFrame::GEOAttributes::hideGEOAttributes() {
    // hide FXGroupBox
    FXGroupBox::hide();
}


void 
GNEFrame::GEOAttributes::UpdateGEOAttributes() {
    myLongitudeTextField->setText(myAc->getAttribute(SUMO_ATTR_LON).c_str());
    myLatitudeTextField->setText(myAc->getAttribute(SUMO_ATTR_LON).c_str());
    if(GNEAttributeCarrier::parse<bool>(myAc->getAttribute(SUMO_ATTR_GEO))) {
        myUseGEOCheckButton->setCheck(TRUE);
    } else {
        myUseGEOCheckButton->setCheck(FALSE);
    }
}


std::map<SumoXMLAttr, std::string> 
GNEFrame::GEOAttributes::getGEOAttributes() const {
    std::map<SumoXMLAttr, std::string> attributes;
    attributes[SUMO_ATTR_LON] = myLongitudeTextField->getText().text();
    attributes[SUMO_ATTR_LAT] = myLatitudeTextField->getText().text();
    if(myUseGEOCheckButton->getCheck()) {
        attributes[SUMO_ATTR_GEO] = "true";
    } else {
        attributes[SUMO_ATTR_GEO] = "false";
    }
    return attributes;
}


long 
GNEFrame::GEOAttributes::onCmdSetLongitude(FXObject*, FXSelector, void*) {
    return 0;
}


long 
GNEFrame::GEOAttributes::onCmdSetLatitude(FXObject*, FXSelector, void*) {
    return 0;
}


long 
GNEFrame::GEOAttributes::onCmdUseGEOParameters(FXObject*, FXSelector, void*) {
    if (myUseGEOCheckButton->getCheck()) {
        myUseGEOCheckButton->setText("true");
    } else {
        myUseGEOCheckButton->setText("false");
    }
    return 1;
}


long 
GNEFrame::GEOAttributes::onCmdHelp(FXObject*, FXSelector, void*) {
    return 0;
}

// ---------------------------------------------------------------------------
// GNEFrame - methods
// ---------------------------------------------------------------------------

GNEFrame::GNEFrame(FXHorizontalFrame* horizontalFrameParent, GNEViewNet* viewNet, const std::string& frameLabel) :
    FXVerticalFrame(horizontalFrameParent, GUIDesignAuxiliarFrame),
    myViewNet(viewNet) {
    // Create font
    myFrameHeaderFont = new FXFont(getApp(), "Arial", 14, FXFont::Bold),

    // Create frame for header
    myHeaderFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);

    // Create frame for left elements of header (By default unused)
    myHeaderLeftFrame = new FXHorizontalFrame(myHeaderFrame, GUIDesignAuxiliarHorizontalFrame);
    myHeaderLeftFrame->hide();

    // Create titel frame
    myFrameHeaderLabel = new FXLabel(myHeaderFrame, frameLabel.c_str(), 0, GUIDesignLabelLeft);

    // Create frame for right elements of header (By default unused)
    myHeaderRightFrame = new FXHorizontalFrame(myHeaderFrame, GUIDesignAuxiliarHorizontalFrame);
    myHeaderRightFrame->hide();

    // Add separator
    new FXHorizontalSeparator(this, GUIDesignHorizontalSeparator);

    // Create frame for contents
    myScrollWindowsContents = new FXScrollWindow(this, GUIDesignContentsScrollWindow);

    // Create frame for contents
    myContentFrame = new FXVerticalFrame(myScrollWindowsContents, GUIDesignContentsFrame);

    // Set font of header
    myFrameHeaderLabel->setFont(myFrameHeaderFont);

    // Hide Frame
    FXVerticalFrame::hide();
}


GNEFrame::~GNEFrame() {
    delete myFrameHeaderFont;
}


void
GNEFrame::focusUpperElement() {
    myFrameHeaderLabel->setFocus();
}


void
GNEFrame::show() {
    // show scroll window
    FXVerticalFrame::show();
    // Show and update Frame Area in which this GNEFrame is placed
    myViewNet->getViewParent()->showFramesArea();
}


void
GNEFrame::hide() {
    // hide scroll window
    FXVerticalFrame::hide();
    // Hide Frame Area in which this GNEFrame is placed
    myViewNet->getViewParent()->hideFramesArea();
}


void
GNEFrame::setFrameWidth(int newWidth) {
    setWidth(newWidth);
    myScrollWindowsContents->setWidth(newWidth);
}


GNEViewNet*
GNEFrame::getViewNet() const {
    return myViewNet;
}


FXLabel*
GNEFrame::getFrameHeaderLabel() const {
    return myFrameHeaderLabel;
}


FXFont*
GNEFrame::getFrameHeaderFont() const {
    return myFrameHeaderFont;
}


/****************************************************************************/
