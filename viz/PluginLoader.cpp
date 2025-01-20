#include "PluginLoader.hpp"

namespace vizkit3d {
        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        QStringList* QtPluginVizkitPCL::getAvailablePlugins() const
        {
            QStringList *pluginNames = new QStringList();
            pluginNames->push_back("PCLPointCloudVisualization");
            pluginNames->push_back("PolygonMeshVisualization");
            return pluginNames;
        }

        QObject* QtPluginVizkitPCL::createPlugin(QString const& pluginName)
        {
            vizkit3d::VizPluginBase* plugin = 0;
            if (pluginName == "PCLPointCloudVisualization")
                plugin = new vizkit3d::PCLPointCloudVisualization();
            else if (pluginName == "PolygonMeshVisualization")
                plugin = new vizkit3d::PolygonMeshVisualization();
            return plugin;
        }

    #if QT_VERSION < 0x050000
        Q_EXPORT_PLUGIN2(QtPluginVizkitBase, QtPluginVizkitBase)
    #endif
}
