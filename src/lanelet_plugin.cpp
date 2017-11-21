#include "lanelet_plugin.hpp"

namespace geodesy {
void fromMsgWithOffset(const geographic_msgs::GeoPoint& from,
                       const geodesy::utmOffset& offset,
                       geodesy::UTMPoint& to,
                       bool normalize) {
    // convert from geographic_msgs::GeoPoint (LatLonAlt) to geodesy::UTMPoint
    // (UTM)
    geodesy::fromMsg(from, to);
    // add offset
    to.altitude = to.altitude + offset.altitude;
    to.easting = to.easting + offset.easting;
    to.northing = to.northing + offset.northing;

    // normalize if canonical grid zone changed due to offset
    if (normalize) {
        auto zone = to.zone;
        auto band = to.band;
        geodesy::normalize(to);
        if (to.zone != zone || to.band != band) {
            ROS_INFO("geodesy::fromMsgWithOffset: UTM grid zone changed after adding "
                     "offset and normalization.");
        }
    } else {
        // warn if point is not in the canoncial grid-zone anymore
        geodesy::UTMPoint normalized(to);
        geodesy::normalize(normalized);
        if (to.zone != normalized.zone || to.band != normalized.band) {
            ROS_WARN("geodesy::fromMsgWithOffset: Point is not in canonical UTM grid "
                     "zone. Current grid zone: %i %c , "
                     "canonical grid zone: %i %c. Normalization is disabled. This may "
                     "lead to inaccuracy and strange "
                     "behavior.",
                     to.zone,
                     to.band,
                     normalized.zone,
                     normalized.band);
        }
    }
}
} // end of namespace geodesy

namespace lanelet_rviz_plugin_ros {

LaneletMapPlugin::LaneletMapPlugin()
        : Display(), tfListener_{tfBuffer_}, mapFileName_(),
          mapFileProperty_("Map-File",
                           "",
                           "Path and filename of the Map (*.osm-File). Ros "
                           "parameters are allowed (${myparam}).",
                           this,
                           SLOT(mapFileChanged()),
                           this),
          referenceFrameProperty_("Reference Frame",
                                  "",
                                  "The Reference Frame. LatLong-coordinates "
                                  "(WGS84) must be provided. Ros parameters "
                                  "are allowed (${myparam}).",
                                  this,
                                  SLOT(referenceFrameChanged()),
                                  this),
          referenceFrameLatProperty_("Reference Frame Latitude",
                                     "",
                                     "The Latitude  of the Reference Frame Origin (WGS84). Either use a "
                                     "Ros parameter "
                                     "(everything except the first given Parameter in the format "
                                     "${myparam} will be "
                                     "ignored), or give the coordinate as decimal value.",
                                     this,
                                     SLOT(referenceFrameChanged()),
                                     this),
          referenceFrameLonProperty_("Reference Frame Longitude",
                                     "",
                                     "The Longitude  of the Reference Frame Origin (WGS84). Either use a "
                                     "Ros parameter "
                                     "(everything except the first given Parameter in the format "
                                     "${myparam} will be "
                                     "ignored), or give the coordinate as decimal value.",
                                     this,
                                     SLOT(referenceFrameChanged())),
          mapVisibilityProperty_("Visibility of Map", true, "", this, SLOT(visibilityPropertyChanged()), this),
          idVisibilityProperty_(
              "Visibility of LaneletIds", true, "", &mapVisibilityProperty_, SLOT(visibilityPropertyChanged()), this),
          seperatorVisibilityProperty_("Visibility of Lanelet Seperators",
                                       true,
                                       "",
                                       &mapVisibilityProperty_,
                                       SLOT(visibilityPropertyChanged()),
                                       this),
          regElementVisibilityProperty_("Visibility of Regulatory Elements (e.g. Stop Lines)",
                                        true,
                                        "",
                                        &mapVisibilityProperty_,
                                        SLOT(visibilityPropertyChanged()),
                                        this),
          laneletWidthProperty_("Linewidth",
                                0.3,
                                "Linewidth (green and red lines)",
                                &mapVisibilityProperty_,
                                SLOT(lineWidthChanged()),
                                this),
          seperatorWidthProperty_("Linewidth Separators",
                                  0.3,
                                  "Linewidth of Lanelet Separators (blue lines)",
                                  &mapVisibilityProperty_,
                                  SLOT(lineWidthChanged()),
                                  this),
          stopLineWidthProperty_("Linewidth Stop Lines",
                                 0.3,
                                 "Linewidth of Stop Lines (red lines)",
                                 &mapVisibilityProperty_,
                                 SLOT(lineWidthChanged()),
                                 this) {
}

LaneletMapPlugin::~LaneletMapPlugin() {
}

void LaneletMapPlugin::reset() {
    Display::reset();
    clear();
}

void LaneletMapPlugin::onInitialize() {
    // call base class initialize function
    Display::onInitialize();
    checkEnvVariables();
}

void LaneletMapPlugin::onEnable() {
    clear();
    // Update Map Filename
    resolveMapFile();
    // Load Map
    loadMap();
}

void LaneletMapPlugin::onDisable() {
    clear();
}

void LaneletMapPlugin::clear() {
    mapElement_.reset();
    clearStatuses();
}

void LaneletMapPlugin::fixedFrameChanged() {
    // Call Base Class method
    Display::fixedFrameChanged();
}

void LaneletMapPlugin::loadMap() {
    // (re)load Map
    if (updateFrames() && checkEnvVariables() && checkMapFile()) {
        createMapObject();
    }
}

/*
 * Load Map and and attach visible Map-Object to the scene_node.
 */
void LaneletMapPlugin::createMapObject() {
    // try to load map and create map object
    try {
        // create Map Element. It is attached to the scene_node on creation
        mapElement_ = std::make_unique<MapElement>(scene_manager_,
                                                   scene_node_,
                                                   mapFileName_,
                                                   fixedFrameOrigin_,
                                                   static_cast<double>(laneletWidthProperty_.getFloat()),
                                                   static_cast<double>(seperatorWidthProperty_.getFloat()),
                                                   static_cast<double>(stopLineWidthProperty_.getFloat()));
    } catch (std::exception& e) {
        setStatus(rviz::StatusProperty::Error, QString("Map"), QString("Error during map creation: ") + e.what());
        return;
    }
    // Set Visibility for individual Map Elements
    visibilityPropertyChanged();

    ROS_INFO("RVIZ:lanelet_plugin: Map loaded. Reference Frame Origin: lat: %.12f "
             "lon: %.12f ; Set Fixed Frame "
             "Origin: UTM "
             "grid zone: %i%c easting: %.12f northing: %.12f",
             referenceFrameOrigin_.latitude,
             referenceFrameOrigin_.longitude,
             fixedFrameOrigin_.zone,
             fixedFrameOrigin_.band,
             fixedFrameOrigin_.easting,
             fixedFrameOrigin_.northing);
    setStatus(rviz::StatusProperty::Ok, QString("Map"), "Map loaded successfully");
    return;
}

void LaneletMapPlugin::mapFileChanged() {
    clear();
    resolveMapFile();
    loadMap();
}

void LaneletMapPlugin::resolveMapFile() {
    mapFileName_ = mapFileProperty_.getStdString();
    // resolve parameters
    try {
        resolveParameters(mapFileName_);
    } catch (std::exception& e) {
        setStatus(rviz::StatusProperty::Error,
                  QString("Map"),
                  QString("Error resolving map filename (or parts) from parameter server."));
        return;
    }
}

void LaneletMapPlugin::referenceFrameChanged() {
    clear();
    loadMap();
}

bool LaneletMapPlugin::updateFrames() {
    // reference frame
    referenceFrame_ = referenceFrameProperty_.getStdString();
    // resolve parameters
    try {
        resolveParameters(referenceFrame_);
        deleteStatus(QString("Reference Frame"));
    } catch (std::exception& e) {
        setStatus(rviz::StatusProperty::Error,
                  QString("Reference Frame"),
                  QString("Error resolving reference frame from parameter server."));
        return false;
    }
    // assume altitude at sea level (0.0m)
    referenceFrameOrigin_.altitude = 0.0;
    // get latitude and longitude from parameter server
    try {
        referenceFrameOrigin_.latitude = resolveDoubleParameters(referenceFrameLatProperty_.getStdString());
        referenceFrameOrigin_.longitude = resolveDoubleParameters(referenceFrameLonProperty_.getStdString());
        deleteStatus(QString("Reference Frame Coordinates"));
    } catch (std::exception& e) {
        setStatus(rviz::StatusProperty::Error,
                  QString("Reference Frame Coordinates"),
                  QString("Error resolving coordinates of the reference frame from "
                          "the parameter server. Make Sure that you "
                          "have set the variables on the parameter server or given "
                          "a valid coordinate (xx.yy format)."));
        return false;
    }

    // get the Offset from the Reference Frame to the Fixed Frame in RVIZ
    geodesy::utmOffset offsetToFixedFrame;
    try {
        // lookup transform
        geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(
            referenceFrame_, context_->getFrameManager()->getFixedFrame(), ros::Time(0), ros::Duration(10.0));
        offsetToFixedFrame.easting = transform.transform.translation.x;
        offsetToFixedFrame.northing = transform.transform.translation.y;
        offsetToFixedFrame.altitude = transform.transform.translation.z;
        deleteStatus(QString("Transform"));
    } catch (tf2::TransformException& e) {
        setStatus(rviz::StatusProperty::Error,
                  QString("Transform"),
                  QString("Error receiving Transform from Reference Frame to Fixed Frame") + e.what());
        return false;
    }

    // calculate the Origin coordinates of the fixed Frame
    geodesy::fromMsgWithOffset(referenceFrameOrigin_,
                               offsetToFixedFrame,
                               fixedFrameOrigin_,
                               false); // normalization is disabled. This may lead
                                       // to inaccuracy or strange behavior when
                                       // the reference frame is improperly
                                       // defined.
    return true;
}

void LaneletMapPlugin::visibilityPropertyChanged() {
    if (mapElement_) {
        if (mapVisibilityProperty_.getBool()) {
            mapElement_->enable();
            mapVisibilityProperty_.expand();
            // check if sub-elements are disabled
            if (!idVisibilityProperty_.getBool()) {
                mapElement_->disable(ObjectClassification::LANELETID);
            }
            if (!seperatorVisibilityProperty_.getBool()) {
                mapElement_->disable(ObjectClassification::SEPERATOR);
            }
            if (!regElementVisibilityProperty_.getBool()) {
                mapElement_->disable(ObjectClassification::REGULATORYELEMENT);
            }
        } else {
            mapElement_->disable();
            mapVisibilityProperty_.collapse();
        }
    }
}

void LaneletMapPlugin::lineWidthChanged() {
    loadMap();
}

/*
 * Helper function to resolve ROS-Parameters
 */
void LaneletMapPlugin::resolveParameters(std::string& str) {
    while (str.find("${") != std::string::npos) {
        auto pos = str.find("${");
        if (pos != std::string::npos) {
            auto endpos = str.find("}");
            auto paramName = str.substr(pos + 2, endpos - pos - 2);
            std::string value;
            if (!update_nh_.getParam(paramName, value)) {
                throw std::runtime_error(std::string("Could not get Parameter from Parameter Server: ") +
                                         paramName.c_str());
            }
            str.erase(pos, endpos - pos + 1);
            str.insert(pos, value);
        }
    }
}

/*
 * Helper function to resolve ROS-Parameters
 * Resolves the first given Parameter ${myparam} or tries to interpret the input
 * as double value
 */
double LaneletMapPlugin::resolveDoubleParameters(std::string str) {
    double value;
    std::size_t pos = str.find("${");
    std::size_t endpos = str.find("}");
    if (pos != std::string::npos || endpos != std::string::npos) {
        auto paramName = str.substr(pos + 2, endpos - pos - 2);
        if (!update_nh_.getParam(paramName, value)) {
            throw std::runtime_error(std::string("Could not get Parameter from Parameter Server: ") +
                                     paramName.c_str());
        }
        return value;
    }
    return boost::lexical_cast<double>(str);
}

/*
 * Helper function to check Filename and -Path
 */
bool LaneletMapPlugin::checkMapFile() {
    boost::filesystem::path map_path(mapFileName_);
    if (!boost::filesystem::exists(map_path) || map_path.extension() != ".osm") {
        setStatus(rviz::StatusProperty::Error,
                  "Map",
                  "Failed to find map under specified path(" + QString::fromStdString(mapFileName_) + ")");
        return false;
    }
    return true;
}

bool LaneletMapPlugin::checkEnvVariables() {
    // TODO: check for other compatible (decimal seperator is point) locales.

    std::string pLC_NUMERIC = std::string(getenv("LC_NUMERIC"));

    if (pLC_NUMERIC == "en_US.UTF-8") {
        deleteStatus(QString("LC_NUMERIC"));
        return true;
    } else {
        std::string warnMessage =
            std::string("Environment Variable LC_NUMERIC is not set to set to \"en_US.UTF-8\". LC_NUMERIC=") +
            pLC_NUMERIC + std::string(" This may cause Errors when loading the map. Make sure you use an Environment "
                                      "Variable that matches your used decimal separator.");

        ROS_ERROR("%s", warnMessage.c_str());
        setStatus(rviz::StatusProperty::Error, QString("LC_NUMERIC"), QString(QString::fromStdString(warnMessage)));

        return false;
    }
}

} // namespace lanelet_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lanelet_rviz_plugin_ros::LaneletMapPlugin, rviz::Display)
