/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "map_element_ogre_helper.hpp"

#include <assert.h>
#include <cmath>
#include <tuple>
#include <typeinfo>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/serialization/strong_typedef.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>


namespace lanelet_rviz_plugin_ros {

struct LatLonOrigin {
    double lat;
    double lon;
};

struct VisualizationOptions {
    double characterHeight{1.0};
    double laneletWidth = {1.0};
    bool fillArea = false;
    bool fillParking = true;
    double areaWidth = {0.3};
    double parkingWidth = {0.3};
    double seperatorWidth = {0.5};
    double stopLineWidth = {0.5};
    double trafficLightHeight = {-162.5};
    Ogre::ColourValue colorLeft{Ogre::ColourValue(0.4, 0.4, 0.4, 0.8)};         // gray
    Ogre::ColourValue colorRight{Ogre::ColourValue(0.4, 0.4, 0.4, 0.8)};        // gray
    Ogre::ColourValue colorStopLine{Ogre::ColourValue(1.0, 0.1, 0.1, 1.0)};     // red
    Ogre::ColourValue colorTrafficLight{Ogre::ColourValue(0.4, 0.4, 0.4, 0.8)}; // gray
    Ogre::ColourValue colorSeperator{Ogre::ColourValue(0.1, 0.1, 0.9, 0.8)};    // blue
    Ogre::ColourValue colorArea{Ogre::ColourValue(0.9, 0.5, 0.1, 0.6)};         // orange
    Ogre::ColourValue colorParking{Ogre::ColourValue(0.0, 0.7, 0.3, 0.8)};      // green
};

enum ObjectClassification {
    UNKOWN,
    MAP,
    LANELETID,
    AREA,
    PARKINGAREA,
    SEPERATOR,
    REGULATORYELEMENT,
    STOPLINE,
    TRAFFICLIGHT,
    SPEEDLIMIT
};
// List of possible Object Classifications
static const ObjectClassification regElementClassifications[] = {
    REGULATORYELEMENT, STOPLINE, TRAFFICLIGHT, SPEEDLIMIT}; // List of Classifications that
                                                            // are associated with regulatory
                                                            // Elements
using ClassifiedMovableObject = std::pair<ObjectClassification, Ogre::MovableObject*>;

class MapElement {
public:
    MapElement(Ogre::SceneManager* scene_manager,
               Ogre::SceneNode* parent_node,
               lanelet::LaneletMapConstPtr theMap,
               const VisualizationOptions& visualizationOptions);
    virtual ~MapElement();

    void disable();
    void enable();

    void disable(ObjectClassification classification);
    void enable(ObjectClassification classification);

    int manObjCounter_{0};
    bool ogreInitialized_{false}; // to check whether ogre settings have been applied

private:
    void visualizeMap(lanelet::LaneletMapConstPtr theMap);
    void addLaneletToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
    void addAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);
    void addParkingAreaToManualObject(const lanelet::ConstArea& area, Ogre::ManualObject* manual);
    void addSeperatorToManualObject(const lanelet::ConstLanelet& lanelet, Ogre::ManualObject* manual);
    void attachRefLinesToSceneNode(std::vector<lanelet::ConstLineString3d>& stopLines, Ogre::SceneNode* parentNode);
    void attachLaneletIdToSceneNode(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parentNode);
    void attachAreaToSceneNode(const lanelet::ConstArea& area, Ogre::SceneNode* parentNode);
    void addRegulatoryElements(const lanelet::ConstLanelet& lanelet, Ogre::SceneNode* parentNode);
    void attachTrafficLightsToSceneNode(const std::vector<lanelet::ConstPolygon3d>& trafficLights,
                                        Ogre::SceneNode* parentNode);


    ogre_helper::Line ogreLineFromLLetLineString(const lanelet::ConstLineString3d& lineString) const;
    ogre_helper::Line ogreLineFromLLetLineString3D(const lanelet::ConstPolygon3d& Polygon3d) const;
    ogre_helper::Line ogreLineFromLLetPolygon(const lanelet::CompoundPolygon3d& lineString) const;
    ogre_helper::Line ogreLineFromLLetPts(const lanelet::ConstPoints3d& ptsVector) const;
    Ogre::Vector3 ogreVec3FromLLetPoint(const lanelet::ConstPoint3d point) const;
    Ogre::Vector3 ogreVec3FromLLetPointTrafficLights(const lanelet::ConstPoint3d point) const;

    std::vector<ClassifiedMovableObject> objects_;
    Ogre::SceneManager* sceneManager_;
    Ogre::SceneNode* sceneNode_;

    Ogre::MaterialPtr material_;

    const VisualizationOptions visualizationOptions_;
};

} // namespace lanelet_rviz_plugin_ros
