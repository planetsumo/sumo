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
/// @file    GUIGlObject.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Oct 2002
/// @version $Id$
///
// Base class for all objects that may be displayed within the openGL-gui
/****************************************************************************/
#ifndef GUIGlObject_h
#define GUIGlObject_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <set>
#include "GUIGlObjectTypes.h"
#include <utils/geom/Boundary.h>
#include <utils/common/StdDefs.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringBijection.h>
#include <utils/common/RGBColor.h>


// ===========================================================================
// definitions
// ===========================================================================

typedef unsigned int GUIGlID;

// ===========================================================================
// class declarations
// ===========================================================================

class GUIGlObjectStorage;
class GUIParameterTableWindow;
class GUIMainWindow;
class GUIGLObjectPopupMenu;
class GUISUMOAbstractView;
class GUIVisualizationSettings;
struct GUIVisualizationTextSettings;
#ifdef HAVE_OSG
namespace osg {
class Node;
}
#endif

// ===========================================================================
// class definitions
// ===========================================================================

class GUIGlObject {
public:
    /// @brief associates object types with strings
    static StringBijection<GUIGlObjectType> TypeNames;
    static const GUIGlID INVALID_ID;

    /** @brief Constructor
     *
     * This is the standard constructor that assures that the object is known
     *  and its id is unique. Use it always :-)
     *
     * @param[in] fullName The complete name, including a type-prefix
     * @see GUIGlObjectStorage
     */
    GUIGlObject(GUIGlObjectType type, const std::string& microsimID);

    /** @brief Constructor
     *
     * This constructor should be used only for compound objects, that share
     *  visualization. Use it only if you know what you are doing.
     *
     * @param[in] fullName The complete name, including a type-prefix
     * @see GUIGlObjectStorage
     */
    GUIGlObject(const std::string& prefix, GUIGlObjectType type, const std::string& microsimID);

    /// @brief Destructor
    virtual ~GUIGlObject();

    /// @name Atomar getter methods
    /// @{
    /// @brief Returns the full name appearing in the tool tip
    /// @return This object's typed id
    const std::string& getFullName() const;

    /// @brief Returns the name of the parent object (if any)
    /// @return This object's parent id
    virtual const std::string& getParentName() const;

    /// @brief Returns the numerical id of the object
    /// @return This object's gl-id
    GUIGlID getGlID() const;
    /// @}

    /// @name interfaces to be implemented by derived classes
    /// @{
    /** @brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     */
    virtual GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) = 0;

    /** @brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     */
    virtual GUIParameterTableWindow* getParameterWindow(GUIMainWindow& app, GUISUMOAbstractView& parent) = 0;

    /** @brief Returns an own type parameter window (optional)
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     */
    virtual GUIParameterTableWindow* getTypeParameterWindow(GUIMainWindow& app, GUISUMOAbstractView& parent);

    /// @brief Returns the id of the object as known to microsim
    virtual const std::string& getMicrosimID() const;

    /// @brief Changes the microsimID of the object
    /// @note happens in NETEDIT
    virtual void setMicrosimID(const std::string& newID);

    /// @brief Returns the type of the object as coded in GUIGlObjectType
    /// @see GUIGlObjectType
    GUIGlObjectType getType() const;

    //// @brief Returns the boundary to which the view shall be centered in order to show the object
    virtual Boundary getCenteringBoundary() const = 0;

    /// @brief Draws the object
    /// @param[in] s The settings for the current view (may influence drawing)
    virtual void drawGL(const GUIVisualizationSettings& s) const = 0;
    /// @}

    /** @brief Draws additional, user-triggered visualisations
     * @param[in] parent The view
     * @param[in] s The settings for the current view (may influence drawing)
     */
    virtual void drawGLAdditional(GUISUMOAbstractView* const parent, const GUIVisualizationSettings& s) const;

#ifdef HAVE_OSG
    /// @brief get OSG Node
    osg::Node* getNode() const;

    /// @brief set OSG Node
    void setNode(osg::Node* node);
#endif

    /// @name Parameter table window I/O
    /// @{
    /// @brief Lets this object know a parameter window showing the object's values was opened
    /// @param[in] w The opened parameter window
    void addParameterTable(GUIParameterTableWindow* w);

    /// @brief Lets this object know a parameter window showing the object's values was closed
    /// @param[in] w The closed parameter window
    void removeParameterTable(GUIParameterTableWindow* w);
    /// @}

    /// @brief draw name of item
    void drawName(const Position& pos, const double scale, const GUIVisualizationTextSettings& settings, const double angle = 0) const;

protected:
    /// @name helper methods for building popup-menus
    /// @{
    /** @brief Builds the header
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildPopupHeader(GUIGLObjectPopupMenu* ret, GUIMainWindow& app, bool addSeparator = true);

    /** @brief Builds an entry which allows to center to the object
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildCenterPopupEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);

    /** @brief Builds entries which allow to copy the name / typed name into the clipboard
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildNameCopyPopupEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);

    /** @brief Builds an entry which allows to (de)select the object
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildSelectionPopupEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);

    /** @brief Builds an entry which allows to open the parameter window
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildShowParamsPopupEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);

    /** @brief Builds an entry which allows to open the type parameter window
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildShowTypeParamsPopupEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);

    /** @brief Builds an entry which allows to copy the cursor position
     *   if geo projection is used, also builds an entry for copying the geo-position
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildPositionCopyEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);

    /** @brief Builds an entry which allows to open the manipulator window
     * @param[in, filled] ret The popup menu to add the entry to
     * @param[in] addSeparator Whether a separator shall be added, too
     */
    void buildShowManipulatorPopupEntry(GUIGLObjectPopupMenu* ret, bool addSeparator = true);
    /// @}

protected:
    /// @brief usually names are prefixed by a type-specific string. this method can be used to change the default
    void setPrefix(const std::string& prefix);

    /// @brief build basic shape popup options. Used to unify pop-ups menu in netedit and SUMO-GUI
    void buildShapePopupOptions(GUIMainWindow& app, GUIGLObjectPopupMenu* ret, const std::string &type);

    /// @brief build basic additional popup options. Used to unify pop-ups menu in netedit and SUMO-GUI
    void buildAdditionalsPopupOptions(GUIMainWindow& app, GUIGLObjectPopupMenu* ret, const std::string &type);

private:
    /// @brief The numerical id of the object
    GUIGlID myGlID;

    /// @brief The type of the object
    const GUIGlObjectType myGLObjectType;

    /// @brief ID of GL object
    std::string myMicrosimID;

    /// @brief prefix of GL Object
    std::string myPrefix;

    /// @brief full name of GL Object
    std::string myFullName;

    /// @brief Parameter table windows which refer to this object
    std::set<GUIParameterTableWindow*> myParamWindows;

    /// @brief create full name
    std::string createFullName() const;

#ifdef HAVE_OSG
    /// @brief OSG Node of this GL object
    osg::Node* myOSGNode;
#endif

    /// @brief LinkStates (Currently unused)
    // static StringBijection<SumoXMLLinkStateValue> LinkStates;

    /// @brief vector for TypeNames Initializer
    static StringBijection<GUIGlObjectType>::Entry GUIGlObjectTypeNamesInitializer[];

private:
    /// @brief Invalidated copy constructor.
    GUIGlObject(const GUIGlObject&);

    /// @brief Invalidated assignment operator.
    GUIGlObject& operator=(const GUIGlObject&);
};
#endif

/****************************************************************************/

