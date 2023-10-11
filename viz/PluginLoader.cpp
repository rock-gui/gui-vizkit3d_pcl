#include "PluginLoader.hpp"

#include "PCLPointCloudVisualization.hpp"
#include "PolygonMeshVisualization.hpp"

namespace vizkit3d {

    /**
    * Returns a list of all available visualization plugins.
    * @return list of plugin names
    */
    QStringList* QtPluginVizkitBase::getAvailablePlugins() const {
        QStringList *pluginNames = new QStringList();
        pluginNames->push_back("PCLPointCloudVisualization");
        pluginNames->push_back("PolygonMeshVisualization");
        return pluginNames;
    }

    QObject* QtPluginVizkitBase::createPlugin(QString const& pluginName) {
        if (pluginName == "PCLPointCloudVisualization")
            return new vizkit3d::PCLPointCloudVisualization();
        else if (pluginName == "PolygonMeshVisualization")
            return new vizkit3d::PolygonMeshVisualization();

        return nullptr;
    }
    #if QT_VERSION < 0x050000
        Q_EXPORT_PLUGIN2(QtPluginVizkitBase, QtPluginVizkitBase)
    #endif
}
