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
#include <tuple>
#include <typeinfo>
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

#include <sim_lanelet/BoundingBox.hpp>
#include <sim_lanelet/Lanelet.hpp>
#include <sim_lanelet/LaneletMap.hpp>
#include <sim_lanelet/RegulatoryElement.hpp>
#include <sim_lanelet/lanelet_point.hpp>
#include <sim_lanelet/llet_xml.hpp>
#include <util_geo_coordinates_ros/util_geo_coordinates_ros.hpp>


namespace lanelet_rviz_plugin_ros {

struct LatLonOrigin {
    double lat;
    double lon;
};

enum ObjectClassification { UNKOWN, MAP, LANELETID, SEPERATOR, REGULATORYELEMENT, STOPLINE, SPEEDLIMIT };
// List of possible Object Classifications
static const ObjectClassification regElementClassifications[] = {
    REGULATORYELEMENT, STOPLINE, SPEEDLIMIT}; // List of Classifications that
                                              // are associated with regulatory
                                              // Elements
using ClassifiedMovableObject = std::pair<ObjectClassification, Ogre::MovableObject*>;

class MapElement {
public:
    MapElement(Ogre::SceneManager* scene_manager,
               Ogre::SceneNode* parent_node,
               std::string map_file,
               std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr,
               const LatLonOrigin& latLonOrigin,
               double laneletWidth = 1.0,
               double seperatorWidth = 0.5,
               double stopLineWidth = 0.5);
    virtual ~MapElement();

    void disable();
    void enable();

    void disable(ObjectClassification classification);
    void enable(ObjectClassification classification);

    int manObjCounter_{0};
    bool ogreInitialized_{false}; // to check whether ogre settings have been applied

private:
    void loadMap(const std::string& map_file);
    void addLaneletToManualObject(const LLet::lanelet_ptr_t& lanelet, Ogre::ManualObject* manual);
    void addSeperatorToManualObject(const LLet::lanelet_ptr_t& lanelet, Ogre::ManualObject* manual);
    void attachStopLinesToSceneNode(const std::vector<LLet::member_variant_t>& stopLines, Ogre::SceneNode* parentNode);
    void attachLaneletIdToSceneNode(const LLet::lanelet_ptr_t& lanelet, Ogre::SceneNode* parentNode);
    void addRegulatoryElements(const LLet::lanelet_ptr_t& lanelet, Ogre::SceneNode* parentNode);

    ogre_helper::Line ogreLineFromLLetPts(const std::vector<LLet::point_with_id_t>& ptsVector);
    Ogre::Vector3 ogreVec3FromLatLon(double lat, double lon);
    Ogre::Vector3 ogreVec3FromLLetPoint(LLet::point_with_id_t point);

    std::vector<ClassifiedMovableObject> objects_;
    Ogre::SceneManager* sceneManager_;
    Ogre::SceneNode* sceneNode_;

    Ogre::MaterialPtr material_;

    std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr_;
    double latLonOriginX_, latLonOriginY_;

    // line settings
    const Ogre::ColourValue colorLeft_ = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);      // red
    const Ogre::ColourValue colorRight_ = Ogre::ColourValue(0.0, 1.0, 0.0, 1.0);     // green
    const Ogre::ColourValue colorSeperator_ = Ogre::ColourValue(0.0, 0.0, 1.0, 1.0); // blue
    const Ogre::ColourValue colorStopLine_ = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);  // red
    double laneletWidth_;
    double seperatorWidth_;
    double stopLineWidth_;
};

} // namespace lanelet_rviz_plugin_ros
