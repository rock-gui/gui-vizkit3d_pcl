#include "PluginLoader.hpp"

namespace vizkit3d {
        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        QStringList* QtPluginVizkit::getAvailablePlugins() const
        {
            QStringList *pluginNames = new QStringList();
            pluginNames->push_back("PCLPointCloudVisualization");
            pluginNames->push_back("PolygonMeshVisualization");
            return pluginNames;
        }

        QObject* QtPluginVizkit::createPlugin(QString const& pluginName)
        {
            vizkit3d::VizPluginBase* plugin = 0;
            if (pluginName == "PCLPointCloudVisualization")
                plugin = new vizkit3d::PCLPointCloudVisualization();
            else if (pluginName == "PolygonMeshVisualization")
                plugin = new vizkit3d::PolygonMeshVisualization();
            return plugin;
        }
}
