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

#include <boost/filesystem.hpp>
#include <boost/exception/all.hpp>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf2_ros/transform_listener.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <util_geo_coordinates_ros/util_geo_coordinates_ros.hpp>

#include "map_element.hpp"


namespace lanelet_rviz_plugin_ros {

class LaneletMapPlugin : public rviz::Display {
    Q_OBJECT
public:
    LaneletMapPlugin();
    virtual ~LaneletMapPlugin();

    // Overrides from Display
    virtual void reset();

protected:
    // Overrides from Display
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void fixedFrameChanged();

private Q_SLOTS:
    void mapFileChanged();
    void visibilityPropertyChanged();
    void referenceFrameChanged();
    void lineWidthChanged();

private:
    void clear();
    void loadMap();
    void createMapObject();
    bool checkMapFile();
    void resolveParameters(std::string& str);
    void resolveMapFile();
    void resolveNavSatFixTopic();
    void createGeoCoordinateTransform();
    bool checkEnvVariables();

    // tf2-listener
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    // map
    std::string mapFileName_;
    std::string navSatFixTopic_;

    std::string originFrameId_;
    LatLonOrigin latLonOrigin_;
    std::unique_ptr<MapElement> mapElement_;
    std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr_;

    // RVIZ-properties
    rviz::StringProperty mapFileProperty_;
    rviz::RosTopicProperty navSatFixTopicProperty_;

    rviz::BoolProperty mapVisibilityProperty_;
    rviz::BoolProperty idVisibilityProperty_;
    rviz::BoolProperty seperatorVisibilityProperty_;
    rviz::BoolProperty regElementVisibilityProperty_;
    rviz::FloatProperty laneletWidthProperty_;
    rviz::FloatProperty seperatorWidthProperty_;
    rviz::FloatProperty stopLineWidthProperty_;
};
} // namespace lanelet_rviz_plugin_ros
