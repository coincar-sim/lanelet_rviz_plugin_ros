#include "map_element.hpp"
#include <simulation_utils/util_rviz.hpp>

namespace lanelet_rviz_plugin_ros {

int MapElement::manObjCounter_ = 0;
bool MapElement::ogreInitialized_ = false;

// visitors for boost::variant to check type
// TODO: transfer them in utils-library
struct is_lanelet_ptr_t : public boost::static_visitor<bool> {
    bool operator()(LLet::lanelet_ptr_t p) const {
        return true;
    }
    bool operator()(LLet::strip_ptr_t p) const {
        return false;
    }
    bool operator()(LLet::point_with_id_t p) const {
        return false;
    }
};
struct is_strip_ptr_t : public boost::static_visitor<bool> {
    bool operator()(LLet::lanelet_ptr_t p) const {
        return false;
    }
    bool operator()(LLet::strip_ptr_t p) const {
        return true;
    }
    bool operator()(LLet::point_with_id_t p) const {
        return false;
    }
};
struct is_point_with_id_t : public boost::static_visitor<bool> {
    bool operator()(LLet::lanelet_ptr_t p) const {
        return false;
    }
    bool operator()(LLet::strip_ptr_t p) const {
        return false;
    }
    bool operator()(LLet::point_with_id_t p) const {
        return true;
    }
};

struct getVariantId : public boost::static_visitor<int> {
    int operator()(LLet::lanelet_ptr_t p) const {
        return p->id();
    }
    int operator()(LLet::strip_ptr_t p) const {
        return 0;
    } // a LineStrip has no Id by itself. return zero
    int operator()(LLet::point_with_id_t p) const {
        return std::get<2>(p);
    }
};

MapElement::MapElement(Ogre::SceneManager* scene_manager,
                       Ogre::SceneNode* parent_node,
                       std::string map_file,
                       geodesy::UTMPoint fixedFrameOrigin,
                       double laneletWidth,
                       double seperatorWidth,
                       double stopLineWidth)
        : sceneManager_(scene_manager), sceneNode_(parent_node->createChildSceneNode()),
          fixedFrameOrigin_(fixedFrameOrigin), laneletWidth_(laneletWidth), seperatorWidth_(seperatorWidth),
          stopLineWidth_(stopLineWidth) {
    if (!ogreInitialized_) {
        // create Material
        material_ = Ogre::getMaterial("lanelet_material");
        // material settings
        material_->getTechnique(0)->setLightingEnabled(
            false); // appearance does not change with different scene orientation
        material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE); // map object "overlays" existing scene
        material_->getTechnique(0)->setDepthWriteEnabled(true);
        material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT + Ogre::TVC_DIFFUSE);
        material_->getTechnique(0)->setCullingMode(Ogre::CULL_NONE); // No culling

        ogreInitialized_ = true;
    }

    // load map and create + attach visible Ogre object
    loadMap(map_file);
}

MapElement::~MapElement() {
    // detach and destroy all objects
    for (auto object : objects_) {
        switch (object.first) {
        case ObjectClassification::MAP: {
            auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
            if (man_object) {
                man_object->getParentSceneNode()->detachObject(man_object);
                sceneManager_->destroyManualObject(man_object);
            }
            break;
        }

        case ObjectClassification::LANELETID: {
            auto mov_text = dynamic_cast<rviz::MovableText*>(object.second);
            if (mov_text) {
                mov_text->getParentSceneNode()->detachObject(mov_text);
                delete mov_text;
            }
            break;
        }

        case ObjectClassification::SEPERATOR: {
            auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
            if (man_object) {
                man_object->getParentSceneNode()->detachObject(man_object);
                sceneManager_->destroyManualObject(man_object);
            }
            break;
        }

        case ObjectClassification::STOPLINE: {
            auto man_object = dynamic_cast<Ogre::ManualObject*>(object.second);
            if (man_object) {
                man_object->getParentSceneNode()->detachObject(man_object);
                sceneManager_->destroyManualObject(man_object);
            }
            break;
        }

        default: {
            if (object.second) {
                object.second->getParentSceneNode()->detachObject(object.second);
                delete object.second;
            }
        }
        }
    }
    // destroy all child scene nodes
    sceneNode_->removeAndDestroyAllChildren();
    // self detach from parent_scene_node
    sceneNode_->getParentSceneNode()->removeChild(sceneNode_);
    // delete sceneNode_
    sceneNode_->getCreator()->destroySceneNode(sceneNode_);
}

void MapElement::disable() {
    sceneNode_->setVisible(false);
}

void MapElement::enable() {
    sceneNode_->setVisible(true);
}

void MapElement::disable(ObjectClassification classification) {
    if (classification == REGULATORYELEMENT) {
        for (auto object : objects_) {
            if (std::find(std::begin(regElementClassifications), std::end(regElementClassifications), object.first) !=
                std::end(regElementClassifications)) {
                object.second->setVisible(false);
            }
        }
    } else {
        for (auto object : objects_) {
            if (object.first == classification) {
                object.second->setVisible(false);
            }
        }
    }
}

void MapElement::enable(ObjectClassification classification) {
    if (classification == REGULATORYELEMENT) {
        for (auto object : objects_) {
            if (std::find(std::begin(regElementClassifications), std::end(regElementClassifications), object.first) !=
                std::end(regElementClassifications)) {
                object.second->setVisible(true);
            }
        }
    } else {
        for (auto object : objects_) {
            if (object.first == classification) {
                object.second->setVisible(true);
            }
        }
    }
}

void MapElement::loadMap(const std::string& map_file) {
    // try to load map and create objects
    boost::filesystem::path map_path(map_file);
    // check for correct filename
    if (!boost::filesystem::exists(map_path) || map_path.extension() != ".osm") {
        std::string errorMessage = std::string("RVIZ::lanelet_rviz_plugin_ros: Failed to "
                                               "find map under specified path:") +
                                   map_file.c_str();
        throw std::runtime_error(errorMessage);
    }
    // Load Map from File
    const LLet::LaneletMap theMap = LLet::LaneletMap(map_file);

    // create manual objects that will be detached to the scene-node
    Ogre::ManualObject* mapManualObject =
        sceneManager_->createManualObject("object_" + std::to_string(manObjCounter_++));
    Ogre::ManualObject* seperatorManualObject =
        sceneManager_->createManualObject("object_" + std::to_string(manObjCounter_++));

    // Attach Lanelet to manual object
    mapManualObject->begin("lanelet_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    seperatorManualObject->begin("lanelet_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    // iterate through all lanelets in map-graph
    BOOST_FOREACH (const auto& vertex, boost::vertices(theMap.graph())) {
        addLaneletToManualObject(theMap.graph()[vertex].lanelet, mapManualObject);
        addSeperatorToManualObject(theMap.graph()[vertex].lanelet, seperatorManualObject);
        addRegulatoryElements(theMap.graph()[vertex].lanelet, sceneNode_);
        attachLaneletIdToSceneNode(theMap.graph()[vertex].lanelet, sceneNode_);
    }
    mapManualObject->end();
    seperatorManualObject->end();

    // attach manual object to scene node
    if (mapManualObject->getNumSections()) {
        sceneNode_->attachObject(mapManualObject);
        // save ptr to manual object (needed later for deletion)
        objects_.push_back(std::make_pair(ObjectClassification::MAP, mapManualObject));
    }

    if (seperatorManualObject->getNumSections()) {
        sceneNode_->attachObject(seperatorManualObject);
        // save ptr to manual object (needed later for deletion)
        objects_.push_back(std::make_pair(ObjectClassification::SEPERATOR, seperatorManualObject));
    }
}

void MapElement::addRegulatoryElements(const LLet::lanelet_ptr_t& lanelet, Ogre::SceneNode* parentNode) {
    // create child SceneNode. Only SceneNodes can be positioned.
    Ogre::SceneNode* regulatoryElementsNode = parentNode->createChildSceneNode();
    // get pointer to regulatory elements
    auto regulatoryElements = lanelet->regulatory_elements();
    // loop  over the regulatory elements
    for (auto&& regElement : regulatoryElements) {
        // check the type of the regulatory Element
        // for each type a function to create the objects has to be implemented
        // seperately
        // because every regulatory element might be defined differently
        // TODO: Add some generic regulatory Element (e.g. dummy-element for not yet
        // implemented types)

        // Stop Lines
        attachStopLinesToSceneNode(regElement->members("stop_line"), regulatoryElementsNode);

        // TODO: Add other regulatory Elements
    }
}

void MapElement::attachStopLinesToSceneNode(const std::vector<LLet::member_variant_t>& stopLines,
                                            Ogre::SceneNode* parentNode) {
    // Create Manual Object, StopLines will be created as Manual Object using the
    // Ogre::drawLine helper function
    Ogre::ManualObject* stopLinesManualObject =
        sceneManager_->createManualObject("object_" + std::to_string(manObjCounter_++));
    stopLinesManualObject->begin("lanelet_material", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (auto&& stopLine : stopLines) {
        // check expected type of variant-container
        if (boost::apply_visitor(is_strip_ptr_t(), stopLine)) {
            // create the actual line
            auto line = lineFromPts(boost::get<LLet::strip_ptr_t>(stopLine)->pts());
            Ogre::drawLine(line, stopLinesManualObject, colorStopLine_, stopLineWidth_);
        } else {
            std::string errorMessage = std::string("RVIZ::lanelet_rviz_plugin_ros: Content-Type of Regulatory Element "
                                                   "Variant is not of expected type.");
            errorMessage += std::string(" ID (0 for LineStrip without ID): ");
            errorMessage += std::to_string(boost::apply_visitor(getVariantId(), stopLine));
            throw std::runtime_error(errorMessage);
        }
    }
    if (stopLinesManualObject->getNumSections()) {
        parentNode->attachObject(stopLinesManualObject);
        // save ptr to manual object (needed later for deletion)
        objects_.push_back(std::make_pair(ObjectClassification::STOPLINE, stopLinesManualObject));
    }
    stopLinesManualObject->end();
}

void MapElement::attachLaneletIdToSceneNode(const LLet::lanelet_ptr_t& lanelet, Ogre::SceneNode* parentNode) {
    // create child SceneNode. Only SceneNodes can be positioned.
    Ogre::SceneNode* childNode = parentNode->createChildSceneNode();

    rviz::MovableText* msg = new rviz::MovableText(std::to_string(lanelet->id()));
    msg->setCharacterHeight(2.0); // TODO: create property and read from the property
    msg->setColor(Ogre::ColourValue::White);
    msg->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE); // Center horizontally and
                                                                                    // display above the node

    Ogre::Vector3 trans = latLongPointToMeterPoint((std::get<0>(lanelet->bounds())->pts().front()));
    util_rviz::setSafePosition(childNode, trans);

    childNode->attachObject(msg);
    objects_.push_back(std::make_pair(ObjectClassification::LANELETID, msg));
}

void MapElement::addLaneletToManualObject(const LLet::lanelet_ptr_t& lanelet, Ogre::ManualObject* manual) {
    // get line from left Linestrip points
    auto lineLeft = lineFromPts(std::get<0>(lanelet->bounds())->pts());
    // draw line as Ogre Object
    Ogre::drawLine(lineLeft, manual, colorLeft_, laneletWidth_);
    // get line from right Linestrip points
    auto lineRight = lineFromPts(std::get<1>(lanelet->bounds())->pts());
    // draw line as Ogre Object
    Ogre::drawLine(lineRight, manual, colorRight_, laneletWidth_);
}

void MapElement::addSeperatorToManualObject(const LLet::lanelet_ptr_t& lanelet, Ogre::ManualObject* manual) {
    // get line from first Point of the left Linestrip to the first Point of the
    // right Linestrip
    std::vector<LLet::point_with_id_t> pointsF;
    pointsF.push_back(std::get<0>(lanelet->bounds())->pts().front());
    pointsF.push_back(std::get<1>(lanelet->bounds())->pts().front());
    // draw line as Ogre Object
    auto line = lineFromPts(pointsF);
    Ogre::drawLine(line, manual, colorSeperator_, seperatorWidth_);

    // get line from last Point of the left Linestrip to the last Point of the
    // right Linestrip
    std::vector<LLet::point_with_id_t> pointsL;
    pointsL.push_back(std::get<0>(lanelet->bounds())->pts().back());
    pointsL.push_back(std::get<1>(lanelet->bounds())->pts().back());
    // draw line as Ogre Object
    line = lineFromPts(pointsL);
    Ogre::drawLine(line, manual, colorSeperator_, seperatorWidth_);
}

Ogre::Line MapElement::lineFromPts(const std::vector<LLet::point_with_id_t>& ptsVector) {
    Ogre::Line line;
    for (LLet::point_with_id_t point : ptsVector) {
        geographic_msgs::GeoPoint geoPoint;
        geoPoint.latitude = std::get<0>(point);
        geoPoint.longitude = std::get<1>(point);
        geoPoint.altitude = 0.0; // assume sea level (altitude = 0.0m)
        line.push_back(latLongPointToMeterPoint(geoPoint));
    }
    return line;
}

/**
 * transforms lat/long/alt coordinates to xyz-coordinates (in reference to
 * transform_)
 * throws std::domain_error if input is not in the same UTM-Grid-Zone as
 * reference Point
 */
Ogre::Vector3 MapElement::latLongPointToMeterPoint(const geographic_msgs::GeoPoint& geoPoint) {
    // convert WGS84 Point to UTM Point within actual grid-zone
    geodesy::UTMPoint utmPoint;
    geodesy::fromMsg(geoPoint, utmPoint);
    // check for same Grid-Zone
    // TODO: implement some kind of conversion, so that points can be in different
    // grid zones. Until then it is
    // disabled.
    if (!geodesy::sameGridZone(utmPoint, fixedFrameOrigin_)) {
        std::string errorMessage = std::string("RVIZ::lanelet_rviz_plugin_ros: Origin of FixedFrame and Points of "
                                               "Lanelet-Map are not in the same UTM-Grid-Zone.");
        errorMessage += std::string(" FixedFrame: ");
        errorMessage += fixedFrameOrigin_.band;
        errorMessage += std::to_string(fixedFrameOrigin_.zone);
        errorMessage += std::string(" Point: ");
        errorMessage += utmPoint.band;
        errorMessage += std::to_string(utmPoint.zone);
        throw std::runtime_error(errorMessage);
    }
    Ogre::Vector3 xyzPoint;
    using boost::numeric_cast;
    using boost::numeric::bad_numeric_cast;
    xyzPoint = Ogre::Vector3(numeric_cast<float>(utmPoint.easting - fixedFrameOrigin_.easting),
                             numeric_cast<float>(utmPoint.northing - fixedFrameOrigin_.northing),
                             numeric_cast<float>(utmPoint.altitude - fixedFrameOrigin_.altitude));
    return xyzPoint;
}

Ogre::Vector3 MapElement::latLongPointToMeterPoint(LLet::point_with_id_t point) {
    geographic_msgs::GeoPoint geoPoint;
    geoPoint.latitude = std::get<0>(point);
    geoPoint.longitude = std::get<1>(point);
    geoPoint.altitude = 0.0; // assume sea level (altitude = 0.0m)

    return latLongPointToMeterPoint(geoPoint);
}

} // namespace lanelet_rviz_plugin_ros
