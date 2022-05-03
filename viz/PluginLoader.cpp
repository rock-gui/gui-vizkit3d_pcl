#include "vizkit3d/Vizkit3DPlugin.hpp"
#include "PCLPointCloudVisualization.hpp"
#include "PolygonMeshVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
    private:
    public:

        QtPluginVizkit() {
        }

        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        virtual QStringList* getAvailablePlugins() const
        {
            QStringList *pluginNames = new QStringList();
            pluginNames->push_back("PCLPointCloudVisualization");
            pluginNames->push_back("PolygonMeshVisualization");
            return pluginNames;
        }

        virtual QObject* createPlugin(QString const& pluginName)
        {
            vizkit3d::VizPluginBase* plugin = 0;
            if (pluginName == "PCLPointCloudVisualization")
                plugin = new vizkit3d::PCLPointCloudVisualization();
            else if (pluginName == "PolygonMeshVisualization")
                plugin = new vizkit3d::PolygonMeshVisualization();
            return plugin;
        };
    };
    Q_PLUGIN_METADATA(IID "QtPluginVizkit")
}
